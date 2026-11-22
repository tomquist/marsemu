/*
 * Native executor for the C28x core and the CLA (optional accelerator).
 *
 * The Python modules stay the reference: marsemu.c28x.isa decodes (this file
 * only executes what it is handed), marsemu.c28x.semantics defines what every
 * instruction does, and this file mirrors it for speed.  An instruction this
 * file does not implement is executed by its Python closure.  Peripherals,
 * interrupts sources and everything else remain Python; they are reached
 * through callbacks.  tests/test_c28x_fast.py runs both and compares.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define M16 0xFFFFu
#define M32 0xFFFFFFFFu
#define M22 0x3FFFFFu
#define PT_RAM 0
#define PT_IO 1
#define PT_ROM 2
#define PT_CODE 3

/* ------------------------------------------------------------------ keys */
enum {
    K_PY = 0,   /* run the Python closure */
#define X(n) K_##n,
#include "_c28x_keys.h"
#undef X
    K_COUNT
};
static const char *KEY_NAMES[] = {
    "",
#define X(n) #n,
#include "_c28x_keys.h"
#undef X
};

typedef struct {
    int16_t key;      /* internal key; -1 = not decoded */
    uint8_t size, flags;
    uint8_t L, A, S, n, m, B, K, pad;
    uint32_t C;
} Entry;

#define F_REPEAT 1
#define F_HOOK 2
#define F_ILLEGAL 4
#define F_PY 8

#define CACHE_LO 0x10000u
#define CACHE_HI_BASE 0x3D0000u
#define CACHE_HI 0x30000u

typedef struct {
    uint32_t mr[4];
    uint32_t mar[2];
    int nf, zf, tf, luf, lvf, rndf32;
    uint32_t mrpc;
} Cla;

typedef struct {
    PyObject_HEAD
    Cla cla;
    uint32_t acc, p, xt, xar[8], rpc, pc;
    uint32_t sp, dp;
    int sxm, ovm, tc, c, z, n, v, pm, ovc;
    int intm, dbgm, page0, vmap, spa, loop, eallow, idlestat, amode, objmode, m0m1map, xf, arp;
    uint32_t ier, ifr, dbgier, dbgstat;
    uint32_t rptc;
    int rpt_pending, rpt_i, rpt_last, bio, idle_wait;
    int64_t wait_pc, wait_left;
    int halted;
    int64_t cycles;
    uint64_t insn_count;
    int err;
    /* memory */
    Py_buffer membuf, ptbuf;
    uint16_t *mem;
    uint8_t *ptype;
    PyObject *io;          /* list: page -> peripheral object with read16/write16 */
    PyObject *clock;
    PyObject *cb_decode;   /* fn(addr) -> tuple */
    PyObject *cb_py;       /* fn(addr) -> None: run the Python closure */
    PyObject *cb_hook;     /* fn(addr) -> bool */
    PyObject *cb_take;     /* fn(k) -> None: Python takes interrupt k (vector, ack) */
    PyObject *cb_romw;     /* fn(addr, v) or None */
    PyObject *cb_iack;     /* fn(v) or None */
    PyObject *cb_trap;     /* fn(k, ret, as_intr) */
    PyObject *cb_iret;     /* unused: IRET is native */
    int16_t keymap[512];
    Entry *lo, *hi;
} Core;

static PyObject *s_cycles, *s_read16, *s_write16;

/* ---------------------------------------------------------------- helpers */
static inline uint32_t s16v(uint32_t v) { return (v & 0x8000) ? (v | 0xFFFF0000u) : v; }
static inline int32_t si16(uint32_t v) { return (int16_t)(uint16_t)v; }
static inline int32_t si32(uint32_t v) { return (int32_t)v; }

static void sync_clock(Core *c) {
    PyObject *v = PyLong_FromLongLong(c->cycles);
    if (!v) { c->err = 1; return; }
    if (PyObject_SetAttr(c->clock, s_cycles, v) < 0) c->err = 1;
    Py_DECREF(v);
}

static Entry *entry_for(Core *c, uint32_t a) {
    if (a < CACHE_LO) return &c->lo[a];
    if (a >= CACHE_HI_BASE && a < CACHE_HI_BASE + CACHE_HI) return &c->hi[a - CACHE_HI_BASE];
    return NULL;
}

static void invalidate(Core *c, uint32_t a) {
    Entry *e = entry_for(c, a);
    if (e) e->key = -1;
    if (a) { e = entry_for(c, a - 1); if (e) e->key = -1; }
}

static uint32_t io_read(Core *c, uint32_t a) {
    sync_clock(c);
    PyObject *p = PyList_GET_ITEM(c->io, a >> 6);
    PyObject *pa = PyLong_FromUnsignedLong(a);
    PyObject *r = PyObject_CallMethodObjArgs(p, s_read16, pa, NULL);
    Py_XDECREF(pa);
    if (!r) { c->err = 1; return 0; }
    uint32_t v = (uint32_t)PyLong_AsUnsignedLongMask(r) & M16;
    Py_DECREF(r);
    return v;
}

static void io_write(Core *c, uint32_t a, uint32_t v) {
    sync_clock(c);
    PyObject *p = PyList_GET_ITEM(c->io, a >> 6);
    PyObject *pa = PyLong_FromUnsignedLong(a), *pv = PyLong_FromUnsignedLong(v & M16);
    PyObject *r = PyObject_CallMethodObjArgs(p, s_write16, pa, pv, NULL);
    Py_XDECREF(pa); Py_XDECREF(pv);
    if (!r) { c->err = 1; return; }
    Py_DECREF(r);
}

static inline uint32_t rd16(Core *c, uint32_t a) {
    a &= M22;
    if (c->ptype[a >> 6] == PT_IO) return io_read(c, a);
    return c->mem[a];
}

static inline void wr16(Core *c, uint32_t a, uint32_t v) {
    a &= M22;
    uint8_t t = c->ptype[a >> 6];
    v &= M16;
    if (t == PT_RAM) { c->mem[a] = (uint16_t)v; return; }
    if (t == PT_IO) { io_write(c, a, v); return; }
    if (t == PT_CODE) { c->mem[a] = (uint16_t)v; invalidate(c, a); return; }
    if (c->cb_romw != Py_None) {
        sync_clock(c);
        PyObject *r = PyObject_CallFunction(c->cb_romw, "kk", (unsigned long)a, (unsigned long)v);
        if (!r) c->err = 1; else Py_DECREF(r);
    }
}

static inline uint32_t rd32(Core *c, uint32_t a) {
    a &= 0x3FFFFE;
    if (c->ptype[a >> 6] == PT_IO) {
        uint32_t lo = io_read(c, a);
        return lo | (io_read(c, a + 1) << 16);
    }
    return c->mem[a] | ((uint32_t)c->mem[a + 1] << 16);
}

static inline void wr32(Core *c, uint32_t a, uint32_t v) {
    a &= 0x3FFFFE;
    if (c->ptype[a >> 6] == PT_RAM) { c->mem[a] = v & M16; c->mem[a + 1] = v >> 16; return; }
    wr16(c, a, v & M16);
    wr16(c, a + 1, v >> 16);
}

/* ------------------------------------------------------------ status */
static inline uint32_t st0(Core *c) {
    return (uint32_t)(c->sxm | c->ovm << 1 | c->tc << 2 | c->c << 3 | c->z << 4 | c->n << 5 | c->v << 6 | c->pm << 7
                      | (c->ovc & 63) << 10);
}
static inline void set_st0(Core *c, uint32_t v) {
    c->sxm = v & 1; c->ovm = v >> 1 & 1; c->tc = v >> 2 & 1; c->c = v >> 3 & 1;
    c->z = v >> 4 & 1; c->n = v >> 5 & 1; c->v = v >> 6 & 1; c->pm = v >> 7 & 7;
    int o = v >> 10 & 63; c->ovc = (o & 32) ? o - 64 : o;
}
static inline uint32_t st1(Core *c) {
    return (uint32_t)(c->intm | c->dbgm << 1 | c->page0 << 2 | c->vmap << 3 | c->spa << 4 | c->loop << 5
                      | c->eallow << 6 | c->idlestat << 7 | c->amode << 8 | c->objmode << 9 | c->m0m1map << 11
                      | c->xf << 12 | c->arp << 13);
}
static inline void set_st1(Core *c, uint32_t v) {
    c->intm = v & 1; c->dbgm = v >> 1 & 1; c->page0 = v >> 2 & 1; c->vmap = v >> 3 & 1;
    c->spa = v >> 4 & 1; c->loop = v >> 5 & 1; c->eallow = v >> 6 & 1; c->idlestat = v >> 7 & 1;
    c->amode = v >> 8 & 1; c->objmode = v >> 9 & 1; c->m0m1map = v >> 11 & 1; c->xf = v >> 12 & 1;
    c->arp = v >> 13 & 7;
}

static inline void nz16(Core *c, uint32_t v) { c->n = (v >> 15) & 1; c->z = (v & M16) == 0; }
static inline void nz32(Core *c, uint32_t v) { c->n = v >> 31; c->z = v == 0; }

static inline void ovc_count(Core *c, int d) {
    int o = c->ovc + d;
    if (o > 31) o -= 64; else if (o < -32) o += 64;
    c->ovc = o;
}

static uint32_t acc_add(Core *c, uint32_t b, uint32_t cin, int cmode, int ovc) {
    uint32_t a = c->acc;
    uint64_t r64 = (uint64_t)a + b + cin;
    uint32_t carry = (uint32_t)(r64 >> 32);
    uint32_t r = (uint32_t)r64;
    if (cmode == 1) { if (carry) c->c = 1; } else c->c = carry;
    if ((~(a ^ b) & (a ^ r)) & 0x80000000u) {
        c->v = 1;
        if (c->ovm) r = (a & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu;
        else if (ovc) ovc_count(c, (a & 0x80000000u) ? -1 : 1);
    }
    c->acc = r; nz32(c, r);
    return r;
}

static uint32_t acc_sub(Core *c, uint32_t b, uint32_t bin, int cmode, int ovc) {
    uint32_t a = c->acc;
    int64_t r64 = (int64_t)a - (int64_t)b - bin;
    int nob = r64 >= 0;
    uint32_t r = (uint32_t)r64;
    if (cmode == 2) { if (!nob) c->c = 0; } else c->c = nob;
    if (((a ^ b) & (a ^ r)) & 0x80000000u) {
        c->v = 1;
        if (c->ovm) r = (a & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu;
        else if (ovc) ovc_count(c, (a & 0x80000000u) ? -1 : 1);
    }
    c->acc = r; nz32(c, r);
    return r;
}

static uint32_t add16(Core *c, uint32_t a, uint32_t b) {
    uint32_t r = a + b;
    c->c = (r >> 16) & 1;
    r &= M16;
    if ((~(a ^ b) & (a ^ r)) & 0x8000) c->v = 1;
    nz16(c, r);
    return r;
}

static uint32_t sub16(Core *c, uint32_t a, uint32_t b) {
    int32_t r0 = (int32_t)a - (int32_t)b;
    c->c = r0 >= 0;
    uint32_t r = (uint32_t)r0 & M16;
    if (((a ^ b) & (a ^ r)) & 0x8000) c->v = 1;
    nz16(c, r);
    return r;
}

static inline uint32_t pshift_v(Core *c, uint32_t p) {
    int pm = c->pm;
    if (pm == 1) return p;
    if (pm == 0) return p << 1;
    if (pm == 5 && c->amode) return p << 4;
    return (uint32_t)(si32(p) >> (pm - 1));
}
#define PSHIFT(c) pshift_v((c), (c)->p)

static inline uint32_t ext16(Core *c, uint32_t v) {
    if (c->sxm && (v & 0x8000)) return v | 0xFFFF0000u;
    return v;
}

static int cond(Core *c, int k) {
    switch (k) {
    case 0: return !c->z;
    case 1: return c->z;
    case 2: return !c->z && !c->n;
    case 3: return !c->n;
    case 4: return c->n;
    case 5: return c->z || c->n;
    case 6: return c->c && !c->z;
    case 7: return c->c;
    case 8: return !c->c;
    case 9: return !c->c || c->z;
    case 10: { int r = !c->v; c->v = 0; return r; }
    case 11: { int r = c->v; c->v = 0; return r; }
    case 12: return !c->tc;
    case 13: return c->tc;
    case 14: return c->bio;
    default: return 1;
    }
}

/* --------------------------------------------------------- loc fields */
/* effective address of a memory mode (with side effects), or -1 for a register mode */
static int32_t ea(Core *c, uint32_t L, int size) {
    int inc = size == 32 ? 2 : 1;
    uint32_t mode = L >> 6, n = L & 7;
    if (mode == 0) return (int32_t)((c->dp << 6) | (L & 63));
    if (mode == 1) return (int32_t)((c->sp - (L & 63)) & M16);
    if (mode == 3) return (int32_t)((c->xar[n] + ((L >> 3) & 7)) & M22);
    uint32_t sub = (L >> 3) & 7, a;
    switch (sub) {
    case 0: a = c->xar[n]; c->xar[n] = a + inc; return (int32_t)(a & M22);
    case 1: a = c->xar[n] - inc; c->xar[n] = a; return (int32_t)(a & M22);
    case 2: return (int32_t)((c->xar[n] + (c->xar[0] & M16)) & M22);
    case 3: return (int32_t)((c->xar[n] + (c->xar[1] & M16)) & M22);
    case 4: return -1;
    case 5: return -1;   /* registers; *BR0 modes are not supported (Python raises) */
    case 6: a = c->xar[c->arp] & M22; c->arp = n; return (int32_t)a;
    default:
        if (n == 5) { a = c->sp; c->sp = (a + inc) & M16; return (int32_t)a; }
        if (n == 6) { a = (c->sp - inc) & M16; c->sp = a; return (int32_t)a; }
        if (n == 7) {
            a = c->xar[6];
            if ((a & 0xFF) == (c->xar[1] & 0xFF)) c->xar[6] = a & 0xFFFFFF00u;
            else c->xar[6] = (a & 0xFFFF0000u) | ((a + inc) & M16);
            return (int32_t)(a & M22);
        }
        {
            uint32_t r = c->arp; a = c->xar[r];
            if (n == 3) c->xar[r] = a + (c->xar[0] & M16);
            else if (n == 4) c->xar[r] = a - (c->xar[0] & M16);
            else if (n == 1) c->xar[r] = a + inc;
            else if (n == 2) c->xar[r] = a - inc;
            return (int32_t)(a & M22);
        }
    }
}

static uint32_t reg16_get(Core *c, uint32_t L) {
    if ((L >> 3) == 0x14) return c->xar[L & 7] & M16;
    switch (L) {
    case 0xA8: return c->acc >> 16;
    case 0xA9: return c->acc & M16;
    case 0xAA: return c->p >> 16;
    case 0xAB: return c->p & M16;
    case 0xAC: return c->xt >> 16;
    case 0xAD: return c->sp;
    }
    return 0;
}

static void reg16_set(Core *c, uint32_t L, uint32_t v) {
    v &= M16;
    if ((L >> 3) == 0x14) { c->xar[L & 7] = (c->xar[L & 7] & 0xFFFF0000u) | v; return; }
    switch (L) {
    case 0xA8: c->acc = (c->acc & M16) | (v << 16); break;
    case 0xA9: c->acc = (c->acc & 0xFFFF0000u) | v; break;
    case 0xAA: c->p = (c->p & M16) | (v << 16); break;
    case 0xAB: c->p = (c->p & 0xFFFF0000u) | v; break;
    case 0xAC: c->xt = (c->xt & M16) | (v << 16); break;
    case 0xAD: c->sp = v; break;
    }
}

static uint32_t reg32_get(Core *c, uint32_t L) {
    if ((L >> 3) == 0x14) return c->xar[L & 7];
    switch (L) { case 0xA9: return c->acc; case 0xAB: return c->p; case 0xAC: return c->xt; }
    return 0;
}

static void reg32_set(Core *c, uint32_t L, uint32_t v) {
    if ((L >> 3) == 0x14) { c->xar[L & 7] = v; return; }
    switch (L) { case 0xA9: c->acc = v; break; case 0xAB: c->p = v; break; case 0xAC: c->xt = v; break; }
}

static inline uint32_t rdl16(Core *c, uint32_t L) {
    int32_t a = ea(c, L, 16);
    return a < 0 ? reg16_get(c, L) : rd16(c, (uint32_t)a);
}
static inline uint32_t rdl32(Core *c, uint32_t L) {
    int32_t a = ea(c, L, 32);
    return a < 0 ? reg32_get(c, L) : rd32(c, (uint32_t)a);
}
static inline void wrl16(Core *c, uint32_t L, uint32_t v) {
    int32_t a = ea(c, L, 16);
    if (a < 0) reg16_set(c, L, v); else wr16(c, (uint32_t)a, v);
}
/* store with the @AX flag rule */
static inline void st16(Core *c, uint32_t L, uint32_t v) {
    wrl16(c, L, v);
    if (L == 0xA8 || L == 0xA9) nz16(c, v & M16);
}
static inline void st32(Core *c, uint32_t L, uint32_t v) {
    int32_t a = ea(c, L, 32);
    if (a < 0) reg32_set(c, L, v); else wr32(c, (uint32_t)a, v);
    if (L == 0xA9) nz32(c, v);
}

static inline uint32_t ax_get(Core *c, int A) { return A ? c->acc >> 16 : c->acc & M16; }
static inline void ax_set(Core *c, int A, uint32_t v) {
    v &= M16;
    if (A) c->acc = (c->acc & M16) | (v << 16); else c->acc = (c->acc & 0xFFFF0000u) | v;
}

static inline void push16(Core *c, uint32_t v) { wr16(c, c->sp, v); c->sp = (c->sp + 1) & M16; }
static inline uint32_t pop16(Core *c) { c->sp = (c->sp - 1) & M16; return rd16(c, c->sp); }
static inline void push32(Core *c, uint32_t v) { wr32(c, c->sp, v); c->sp = (c->sp + 2) & M16; }
static inline uint32_t pop32(Core *c) { c->sp = (c->sp - 2) & M16; return rd32(c, c->sp); }

static inline uint32_t mul_ss(uint32_t a, uint32_t b) { return (uint32_t)(si16(a) * si16(b)); }

/* ------------------------------------------------------------- interrupts */
static void iret(Core *c) {
    c->pc = pop32(c) & M22;
    uint32_t v = pop32(c);
    c->ier = v & M16; c->dbgstat = v >> 16;
    v = pop32(c);
    uint32_t keep = (uint32_t)(c->loop << 5 | c->idlestat << 7 | c->m0m1map << 11);
    set_st1(c, ((v & M16) & ~((1u << 5) | (1u << 7) | (1u << 11))) | keep);
    c->dp = v >> 16;
    v = pop32(c);
    c->xar[0] = (c->xar[0] & 0xFFFF0000u) | (v & M16);
    c->xar[1] = (c->xar[1] & 0xFFFF0000u) | (v >> 16);
    c->p = pop32(c);
    c->acc = pop32(c);
    v = pop32(c);
    set_st0(c, v & M16);
    c->xt = (c->xt & M16) | (v & 0xFFFF0000u);
    c->sp = (c->sp - 1) & M16;
}

/* --------------------------------------------------------------- decode */
static int decode(Core *c, uint32_t a, Entry *out) {
    PyObject *r = PyObject_CallFunction(c->cb_decode, "k", (unsigned long)a);
    if (!r) return -1;
    long vals[11];
    if (!PyTuple_Check(r) || PyTuple_GET_SIZE(r) != 11) { Py_DECREF(r); PyErr_SetString(PyExc_TypeError, "decode"); return -1; }
    for (int i = 0; i < 11; i++) vals[i] = PyLong_AsLong(PyTuple_GET_ITEM(r, i));
    Py_DECREF(r);
    int py_key = (int)vals[0];
    out->key = (py_key >= 0 && py_key < 512) ? c->keymap[py_key] : K_PY;
    out->size = (uint8_t)vals[1];
    out->L = (uint8_t)vals[2]; out->A = (uint8_t)vals[3]; out->C = (uint32_t)vals[4]; out->S = (uint8_t)vals[5];
    out->n = (uint8_t)vals[6]; out->m = (uint8_t)vals[7]; out->B = (uint8_t)vals[8]; out->K = (uint8_t)vals[9];
    out->flags = (uint8_t)vals[10];
    if (out->flags & (F_HOOK | F_ILLEGAL | F_PY)) out->key = K_PY;
    return 0;
}

/* run the Python closure of the instruction at pc (pc already advanced) */
static int run_py(Core *c, uint32_t at) {
    sync_clock(c);
    PyObject *r = PyObject_CallFunction(c->cb_py, "k", (unsigned long)at);
    if (!r) return -1;
    Py_DECREF(r);
    return 0;
}

/* execute one decoded instruction; returns 0, or -1 on a Python error */
static int exec1(Core *c, Entry *e, uint32_t at);

#include "_c28x_exec.h"

/* ------------------------------------------------------------- the loop */
static PyObject *Core_run(Core *c, PyObject *args) {
    long long budget;
    if (!PyArg_ParseTuple(args, "L", &budget)) return NULL;
    PyObject *cv = PyObject_GetAttr(c->clock, s_cycles);
    if (!cv) return NULL;
    c->cycles = PyLong_AsLongLong(cv);
    Py_DECREF(cv);
    int64_t start = c->cycles, end = start + budget;
    uint64_t n = 0;
    c->err = 0;
    while (c->cycles < end) {
        if (c->ifr && !c->intm) {
            uint32_t bits = c->ifr & c->ier;
            if (bits) {
                int k = __builtin_ctz(bits) + 1;
                sync_clock(c);
                /* Python acknowledges (PIE) and returns the vector; the context save is here */
                c->ifr &= ~(1u << (k - 1));
                PyObject *r = PyObject_CallFunction(c->cb_take, "i", k);
                if (!r) goto fail;
                uint32_t vec = (uint32_t)PyLong_AsUnsignedLongMask(r) & M22;
                Py_DECREF(r);
                uint32_t ret = c->pc;
                c->sp = (c->sp + 1) & M16;
                push32(c, (c->xt & 0xFFFF0000u) | st0(c));
                push32(c, c->acc);
                push32(c, c->p);
                push32(c, ((c->xar[1] & M16) << 16) | (c->xar[0] & M16));
                push32(c, (c->dp << 16) | st1(c));
                push32(c, (c->dbgstat << 16) | c->ier);
                push32(c, ret);
                if (k <= 16) c->ier &= ~(1u << (k - 1));
                c->intm = 1; c->dbgm = 1; c->loop = 0; c->eallow = 0; c->idlestat = 0; c->idle_wait = 0;
                c->pc = vec;
                if (c->err) goto fail;
            }
        }
        if (c->idle_wait || c->halted) break;
        uint32_t pc = c->pc;
        if ((int64_t)pc == c->wait_pc && c->wait_left) break;
        Entry tmp, *e = entry_for(c, pc);
        if (!e) { e = &tmp; e->key = -1; }
        if (e->key < 0 && decode(c, pc, e) < 0) goto fail;
        uint32_t nxt = (pc + e->size) & M22;
        c->pc = nxt;
        if (exec1(c, e, pc) < 0 || c->err) goto fail;
        n++;
        if (c->pc != nxt) c->cycles += 3;
        if (c->rpt_pending) {
            c->rpt_pending = 0;
            uint32_t pc2 = c->pc;
            Entry tmp2, *e2 = entry_for(c, pc2);
            if (!e2) { e2 = &tmp2; e2->key = -1; }
            if (e2->key < 0 && decode(c, pc2, e2) < 0) goto fail;
            c->pc = (pc2 + e2->size) & M22;
            uint32_t cnt = c->rptc;
            if (e2->key == K_NOP && e2->L == 0) {
                c->cycles += cnt + 1;
            } else if ((e2->flags & F_REPEAT) && cnt) {
                for (uint32_t i = 0; i <= cnt; i++) {
                    c->rpt_i = (int)i; c->rpt_last = i == cnt;
                    if (exec1(c, e2, pc2) < 0 || c->err) { c->rpt_i = 0; c->rpt_last = 1; goto fail; }
                }
                c->rpt_i = 0; c->rpt_last = 1;
                c->cycles += cnt + 1;
            } else {
                if (exec1(c, e2, pc2) < 0 || c->err) goto fail;
                c->cycles += 1;
            }
            n++;
            continue;
        }
        c->cycles += 1;
    }
    c->insn_count += n;
    sync_clock(c);
    if (c->err) return NULL;
    return PyLong_FromLongLong(c->cycles - start);
fail:
    c->insn_count += n;
    {
        PyObject *et, *ev, *tb;
        PyErr_Fetch(&et, &ev, &tb);
        sync_clock(c);
        PyErr_Restore(et, ev, tb);
    }
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_RuntimeError, "c28x core error");
    return NULL;
}

/* ------------------------------------------------------------- object */
static int Core_init(Core *c, PyObject *args, PyObject *kw) {
    PyObject *mem, *pt, *keys;
    if (!PyArg_ParseTuple(args, "OOOOOOOOOOOO", &mem, &pt, &c->io, &c->clock, &c->cb_decode, &c->cb_py, &c->cb_hook,
                          &c->cb_take, &c->cb_romw, &c->cb_iack, &c->cb_trap, &keys))
        return -1;
    if (PyObject_GetBuffer(mem, &c->membuf, PyBUF_WRITABLE) < 0) return -1;
    if (PyObject_GetBuffer(pt, &c->ptbuf, PyBUF_WRITABLE) < 0) return -1;
    if (c->membuf.len < (Py_ssize_t)(2u << 22) || c->ptbuf.len < (Py_ssize_t)(1u << 16)) {
        PyErr_SetString(PyExc_ValueError, "memory too small");
        return -1;
    }
    c->mem = (uint16_t *)c->membuf.buf;
    c->ptype = (uint8_t *)c->ptbuf.buf;
    Py_INCREF(c->io); Py_INCREF(c->clock); Py_INCREF(c->cb_decode); Py_INCREF(c->cb_py); Py_INCREF(c->cb_hook);
    Py_INCREF(c->cb_take); Py_INCREF(c->cb_romw); Py_INCREF(c->cb_iack); Py_INCREF(c->cb_trap);
    c->lo = PyMem_Calloc(CACHE_LO, sizeof(Entry));
    c->hi = PyMem_Calloc(CACHE_HI, sizeof(Entry));
    if (!c->lo || !c->hi) { PyErr_NoMemory(); return -1; }
    for (uint32_t i = 0; i < CACHE_LO; i++) c->lo[i].key = -1;
    for (uint32_t i = 0; i < CACHE_HI; i++) c->hi[i].key = -1;
    for (int i = 0; i < 512; i++) c->keymap[i] = K_PY;
    Py_ssize_t nk = PyList_Size(keys);
    for (Py_ssize_t i = 0; i < nk && i < 512; i++) {
        const char *name = PyUnicode_AsUTF8(PyList_GET_ITEM(keys, i));
        if (!name) return -1;
        for (int k = 1; k < K_COUNT; k++)
            if (!strcmp(name, KEY_NAMES[k])) { c->keymap[i] = (int16_t)k; break; }
    }
    c->wait_pc = -1;
    c->rpt_last = 1;
    return 0;
}

static void Core_dealloc(Core *c) {
    if (c->membuf.obj) PyBuffer_Release(&c->membuf);
    if (c->ptbuf.obj) PyBuffer_Release(&c->ptbuf);
    Py_XDECREF(c->io); Py_XDECREF(c->clock); Py_XDECREF(c->cb_decode); Py_XDECREF(c->cb_py);
    Py_XDECREF(c->cb_hook); Py_XDECREF(c->cb_take); Py_XDECREF(c->cb_romw); Py_XDECREF(c->cb_iack);
    Py_XDECREF(c->cb_trap);
    PyMem_Free(c->lo); PyMem_Free(c->hi);
    Py_TYPE(c)->tp_free((PyObject *)c);
}

static PyObject *Core_invalidate(Core *c, PyObject *args) {
    unsigned long a;
    if (!PyArg_ParseTuple(args, "k", &a)) return NULL;
    invalidate(c, (uint32_t)a);
    Py_RETURN_NONE;
}

static PyObject *Core_flush(Core *c, PyObject *noargs) {
    for (uint32_t i = 0; i < CACHE_LO; i++) c->lo[i].key = -1;
    for (uint32_t i = 0; i < CACHE_HI; i++) c->hi[i].key = -1;
    Py_RETURN_NONE;
}

static PyObject *Core_get_xar(Core *c, PyObject *args) {
    int i;
    if (!PyArg_ParseTuple(args, "i", &i) || i < 0 || i > 7) return NULL;
    return PyLong_FromUnsignedLong(c->xar[i]);
}

static PyObject *Core_set_xar(Core *c, PyObject *args) {
    int i; unsigned long v;
    if (!PyArg_ParseTuple(args, "ik", &i, &v) || i < 0 || i > 7) return NULL;
    c->xar[i] = (uint32_t)v;
    Py_RETURN_NONE;
}

static PyObject *Core_iret(Core *c, PyObject *noargs) { iret(c); if (c->err) return NULL; Py_RETURN_NONE; }

static PyObject *Core_supported(Core *c, PyObject *noargs) {
    PyObject *l = PyList_New(0);
    for (int k = 1; k < K_COUNT; k++) {
        PyObject *s = PyUnicode_FromString(KEY_NAMES[k]);
        PyList_Append(l, s);
        Py_DECREF(s);
    }
    return l;
}

static PyMethodDef Core_methods[] = {
    {"run", (PyCFunction)Core_run, METH_VARARGS, "run(cycles) -> cycles used"},
    {"invalidate", (PyCFunction)Core_invalidate, METH_VARARGS, "drop the decoded instruction at an address"},
    {"flush", (PyCFunction)Core_flush, METH_NOARGS, "drop all decoded instructions"},
    {"get_xar", (PyCFunction)Core_get_xar, METH_VARARGS, ""},
    {"set_xar", (PyCFunction)Core_set_xar, METH_VARARGS, ""},
    {"iret", (PyCFunction)Core_iret, METH_NOARGS, ""},
    {"supported", (PyCFunction)Core_supported, METH_NOARGS, "keys executed natively"},
    {NULL}
};

/* register attributes */
#define U32(name) \
    static PyObject *get_##name(Core *c, void *x) { return PyLong_FromUnsignedLong(c->name); } \
    static int set_##name(Core *c, PyObject *v, void *x) { c->name = (uint32_t)PyLong_AsUnsignedLongMask(v); return PyErr_Occurred() ? -1 : 0; }
#define INT(name) \
    static PyObject *get_##name(Core *c, void *x) { return PyLong_FromLong(c->name); } \
    static int set_##name(Core *c, PyObject *v, void *x) { c->name = (int)PyLong_AsLong(v); return PyErr_Occurred() ? -1 : 0; }
#define I64(name) \
    static PyObject *get_##name(Core *c, void *x) { return PyLong_FromLongLong(c->name); } \
    static int set_##name(Core *c, PyObject *v, void *x) { c->name = PyLong_AsLongLong(v); return PyErr_Occurred() ? -1 : 0; }
U32(acc) U32(p) U32(xt) U32(rpc) U32(pc) U32(sp) U32(dp) U32(ier) U32(ifr) U32(dbgier) U32(dbgstat) U32(rptc)
INT(sxm) INT(ovm) INT(tc) INT(c) INT(z) INT(n) INT(v) INT(pm) INT(ovc) INT(intm) INT(dbgm) INT(page0) INT(vmap)
INT(spa) INT(loop) INT(eallow) INT(idlestat) INT(amode) INT(objmode) INT(m0m1map) INT(xf) INT(arp)
INT(rpt_pending) INT(rpt_i) INT(rpt_last) INT(bio) INT(idle_wait) INT(halted)
I64(wait_pc) I64(wait_left)
static PyObject *get_insn_count(Core *c, void *x) { return PyLong_FromUnsignedLongLong(c->insn_count); }
static int set_insn_count(Core *c, PyObject *v, void *x) { c->insn_count = PyLong_AsUnsignedLongLong(v); return 0; }
static PyObject *get_st0(Core *c, void *x) { return PyLong_FromUnsignedLong(st0(c)); }
static int set_st0_(Core *c, PyObject *v, void *x) { set_st0(c, (uint32_t)PyLong_AsUnsignedLongMask(v)); return 0; }
static PyObject *get_st1(Core *c, void *x) { return PyLong_FromUnsignedLong(st1(c)); }
static int set_st1_(Core *c, PyObject *v, void *x) { set_st1(c, (uint32_t)PyLong_AsUnsignedLongMask(v)); return 0; }

#define GS(name) {#name, (getter)get_##name, (setter)set_##name, NULL, NULL},
static PyGetSetDef Core_getset[] = {
    GS(acc) GS(p) GS(xt) GS(rpc) GS(pc) GS(sp) GS(dp) GS(ier) GS(ifr) GS(dbgier) GS(dbgstat) GS(rptc)
    GS(sxm) GS(ovm) GS(tc) GS(c) GS(z) GS(n) GS(v) GS(pm) GS(ovc) GS(intm) GS(dbgm) GS(page0) GS(vmap)
    GS(spa) GS(loop) GS(eallow) GS(idlestat) GS(amode) GS(objmode) GS(m0m1map) GS(xf) GS(arp)
    GS(rpt_pending) GS(rpt_i) GS(rpt_last) GS(bio) GS(idle_wait) GS(halted) GS(wait_pc) GS(wait_left)
    GS(insn_count)
    {"st0", (getter)get_st0, (setter)set_st0_, NULL, NULL},
    {"st1", (getter)get_st1, (setter)set_st1_, NULL, NULL},
    {NULL}
};

static PyTypeObject CoreType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "marsemu.c28x._c28x.Core",
    .tp_basicsize = sizeof(Core),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc)Core_init,
    .tp_dealloc = (destructor)Core_dealloc,
    .tp_methods = Core_methods,
    .tp_getset = Core_getset,
};

#include "_cla.h"

static PyMethodDef module_methods[] = {
    {"cla_run", (PyCFunction)cla_run, METH_VARARGS, "run a CLA task natively"},
    {"cla_state", (PyCFunction)cla_state, METH_VARARGS, "get or set the CLA registers kept in the core"},
    {NULL}
};

static struct PyModuleDef moduledef = {PyModuleDef_HEAD_INIT, "_c28x", "native C28x/CLA executor", -1, module_methods};

PyMODINIT_FUNC PyInit__c28x(void) {
    s_cycles = PyUnicode_InternFromString("cycles");
    s_read16 = PyUnicode_InternFromString("read16");
    s_write16 = PyUnicode_InternFromString("write16");
    if (PyType_Ready(&CoreType) < 0) return NULL;
    PyObject *m = PyModule_Create(&moduledef);
    if (!m) return NULL;
    Py_INCREF(&CoreType);
    PyModule_AddObject(m, "Core", (PyObject *)&CoreType);
    return m;
}
