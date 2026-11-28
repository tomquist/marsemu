/* Native CLA task runner for _c28x.c, mirroring marsemu/c28x/cla.py. */


static inline float cf(uint32_t b) {
    if (!(b & 0x7F800000u)) b &= 0x80000000u;   /* denormals read as zero */
    float f;
    memcpy(&f, &b, 4);
    return f;
}

static inline uint32_t cb(Cla *s, float f) {
    uint32_t b;
    memcpy(&b, &f, 4);
    uint32_t e = b & 0x7F800000u;
    if (e == 0x7F800000u) { s->lvf = 1; return 0x7F7FFFFFu | (b & 0x80000000u); }
    if (!e && (b & 0x7FFFFFu)) { s->luf = 1; return b & 0x80000000u; }
    return b;
}

static inline uint32_t cla_rd16(Core *c, uint32_t a) { return rd16(c, a & 0xFFFF); }
static inline void cla_wr16(Core *c, uint32_t a, uint32_t v) { wr16(c, a & 0xFFFF, v); }
static inline uint32_t cla_rd32(Core *c, uint32_t a) {
    a &= 0xFFFE;
    if (c->ptype[a >> 6] == PT_RAM || c->ptype[a >> 6] == PT_CODE) return c->mem[a] | ((uint32_t)c->mem[a + 1] << 16);
    uint32_t lo = rd16(c, a);
    return lo | (rd16(c, a + 1) << 16);
}
static inline void cla_wr32(Core *c, uint32_t a, uint32_t v) {
    a &= 0xFFFE;
    if (c->ptype[a >> 6] == PT_RAM) { mem_set(c, a, v & M16); mem_set(c, a + 1, v >> 16); return; }
    wr16(c, a, v & M16);
    wr16(c, a + 1, v >> 16);
}

static int cla_cond(int k, int nf, int zf, int tf, int luf, int lvf) {
    switch (k) {
    case 0: return !zf;
    case 1: return zf;
    case 2: return !zf && !nf;
    case 3: return !nf;
    case 4: return nf;
    case 5: return zf || nf;
    case 10: return tf;
    case 11: return !tf;
    case 12: return luf;
    case 13: return lvf;
    case 14: case 15: return 1;
    }
    return 0;
}

static inline void zn_f(Cla *s, uint32_t b) {
    s->zf = !(b & 0x7FFFFFFFu) || !(b & 0x7F800000u);
    s->nf = (b & 0x80000000u) && !s->zf;
}
static inline void zn_i(Cla *s, uint32_t v) { s->zf = v == 0; s->nf = v >> 31; }

static uint32_t sat_i(float x, double lo, double hi, int rnd) {
    if (x != x) return 0;
    double v = rnd ? floor((double)x + 0.5) : trunc((double)x);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint32_t)(int64_t)v;
}

/* effective address of a CLA memory operand (with MAR post-increment) */
static uint32_t cla_ea(Cla *s, int mode, uint32_t lsw) {
    if (mode == 0) return lsw;
    if (mode == 1 || mode == 2) {
        int r = mode - 1;
        uint32_t a = s->mar[r];
        s->mar[r] = (a + lsw) & 0xFFFF;
        return a;
    }
    if (mode == 5 || mode == 6) return (s->mar[mode - 5] + lsw) & 0xFFFF;
    return lsw;
}

#define MRF(i) cf(s->mr[i])

/* executes one instruction; returns 0 normal, 1 MSTOP, 2 branch (kind in *bk), -1 unknown */
static int cla_step(Core *c, Cla *s, uint32_t pc, uint32_t msw, uint32_t lsw, int *mar_reg, uint32_t *mar_val) {
    uint32_t top = msw >> 12;
    *mar_reg = -1;
    if (top <= 6) {
        int ff = (msw >> 10) & 3, ee = (msw >> 8) & 3, dd = (msw >> 6) & 3, aa = (msw >> 4) & 3, mode = msw & 0xF;
        float e = MRF(ee), f = MRF(ff);
        uint32_t ad;
        if (top == 3) {
            float acc = MRF(3) + MRF(2), prod = e * f;
            ad = cla_ea(s, mode, lsw);
            uint32_t v = cla_rd32(c, ad);
            s->mr[3] = cb(s, acc);
            s->mr[dd] = cb(s, prod);
            s->mr[aa] = v;
            zn_f(s, v);
            return 0;
        }
        int base = top < 4 ? top : top - 4;
        float r = base == 0 ? e * f : (base == 1 ? e + f : e - f);
        ad = cla_ea(s, mode, lsw);
        if (top >= 4) {
            cla_wr32(c, ad, s->mr[aa]);
            s->mr[dd] = cb(s, r);
        } else {
            uint32_t v = cla_rd32(c, ad);
            s->mr[dd] = cb(s, r);
            s->mr[aa] = v;
            zn_f(s, v);
        }
        return 0;
    }
    /* the most frequent ones first (no earlier test below matches them) */
    switch (msw & 0xFFFC) {
    case 0x7840: s->mr[msw & 3] = lsw << 16; return 0;                                   /* MMOVIZ */
    case 0x7880: s->mr[msw & 3] = (s->mr[msw & 3] & 0xFFFF0000u) | lsw; return 0;         /* MMOVXI */
    }
    if ((msw & 0xFFE0) == 0x7980 || (msw & 0xFFF0) == 0x79A0) return 2;                   /* branches */
    int a = (msw >> 4) & 3, mode = msw & 0xF;
    if ((msw & 0xFC00) == 0x7000) {
        int cnd = (msw >> 6) & 0xF;
        uint32_t ad = cla_ea(s, mode, lsw);
        if (cla_cond(cnd, s->nf, s->zf, s->tf, s->luf, s->lvf)) {
            uint32_t v = cla_rd32(c, ad);
            s->mr[a] = v;
            if (cnd == 15) zn_f(s, v);
        }
        return 0;
    }
    uint32_t grp = msw & 0xFFC0;
    if (grp >= 0x7400 && grp <= 0x75C0) {
        uint32_t ad = cla_ea(s, mode, lsw), v;
        switch (grp) {
        case 0x7400: v = cla_rd32(c, ad); s->mr[a] = v; cla_wr32(c, ad + 2, v); zn_f(s, v); break;
        case 0x7440: v = cla_rd32(c, ad); s->mr[a] = cb(s, (float)(int32_t)v); break;
        case 0x7480: v = cla_rd32(c, ad); s->mr[a] = cb(s, (float)v); break;
        case 0x74C0: cla_wr32(c, ad, s->mr[a]); break;
        case 0x7500: v = cla_rd16(c, ad); s->mr[a] = cb(s, (float)(int16_t)v); break;
        case 0x7540: v = cla_rd16(c, ad); s->mr[a] = cb(s, (float)v); break;
        case 0x7580: v = cla_rd16(c, ad); s->mr[a] = v; zn_i(s, v); break;
        default: cla_wr16(c, ad, s->mr[a] & M16); break;
        }
        return 0;
    }
    if ((msw & 0xFF00) == 0x7600) {
        int r = (msw >> 6) & 1;
        uint32_t ad = cla_ea(s, mode, lsw);
        if (msw & 0x80) cla_wr16(c, ad, s->mar[r]);
        else { *mar_reg = r; *mar_val = cla_rd16(c, ad); }
        return 0;
    }
    if ((msw & 0xFFF0) == 0x7700) {   /* MMOV32 MSTF,mem32 */
        uint32_t v = cla_rd32(c, cla_ea(s, mode, lsw));
        s->lvf = v & 1; s->luf = v >> 1 & 1; s->nf = v >> 2 & 1; s->zf = v >> 3 & 1; s->tf = v >> 6 & 1;
        s->rndf32 = v >> 9 & 1; s->mrpc = (v >> 12) & 0xFFF;
        return 0;
    }
    if ((msw & 0xFFF0) == 0x7740) {
        uint32_t v = (uint32_t)(s->lvf | s->luf << 1 | s->nf << 2 | s->zf << 3 | s->tf << 6 | s->rndf32 << 9) | (s->mrpc & 0xFFF) << 12;
        cla_wr32(c, cla_ea(s, mode, lsw), v);
        return 0;
    }
    int b2 = (msw >> 2) & 3, a0 = msw & 3;
    float fimm = cf(lsw << 16);
    switch (msw & 0xFFF0) {
    case 0x7780: s->mr[a0] = cb(s, fimm * MRF(b2)); return 0;
    case 0x77C0: s->mr[a0] = cb(s, fimm + MRF(b2)); return 0;
    case 0x7800: s->mr[a0] = cb(s, fimm - MRF(b2)); return 0;
    }
    switch (msw & 0xFFFC) {
    case 0x7840: s->mr[a0] = lsw << 16; return 0;
    case 0x7880: s->mr[a0] = (s->mr[a0] & 0xFFFF0000u) | lsw; return 0;
    case 0x78C0: case 0x7900: case 0x7940: {
        float x = MRF(a0);
        s->zf = x == fimm; s->nf = x < fimm;
        if ((msw & 0xFFFC) == 0x7900 && x < fimm) s->mr[a0] = lsw << 16;
        if ((msw & 0xFFFC) == 0x7940 && x > fimm) s->mr[a0] = lsw << 16;
        return 0; }
    }
    if ((msw & 0xFFE0) == 0x7980 || (msw & 0xFFF0) == 0x79A0) return 2;
    if (msw == 0x79C0) {
        uint32_t fl = lsw >> 8, va = lsw & 0xFF;
        if (fl & 1) s->lvf = va & 1;
        if (fl & 2) s->luf = va >> 1 & 1;
        if (fl & 4) s->nf = va >> 2 & 1;
        if (fl & 8) s->zf = va >> 3 & 1;
        if (fl & 64) s->tf = va >> 6 & 1;
        if (fl & 128) s->rndf32 = va >> 7 & 1;
        return 0;
    }
    int la = lsw & 3, lb = (lsw >> 2) & 3, lc = (lsw >> 4) & 3, ld = (lsw >> 6) & 3, le = (lsw >> 8) & 3, lf = (lsw >> 10) & 3;
    int cndl = (lsw >> 4) & 0xF;
    switch (msw) {
    case 0x7A00: case 0x7A40: {
        float p = MRF(lb) * MRF(lc), q = msw == 0x7A40 ? MRF(le) - MRF(lf) : MRF(le) + MRF(lf);
        s->mr[la] = cb(s, p); s->mr[ld] = cb(s, q); return 0; }
    case 0x7A80: { uint32_t v = s->mr[lb];
        if (cla_cond(cndl, s->nf, s->zf, s->tf, s->luf, s->lvf)) v ^= 0x80000000u;
        s->mr[la] = v; if (cndl == 15) zn_f(s, v); return 0; }
    case 0x7AC0:
        if (cla_cond(cndl, s->nf, s->zf, s->tf, s->luf, s->lvf)) { s->mr[la] = s->mr[lb]; if (cndl == 15) zn_f(s, s->mr[la]); }
        return 0;
    case 0x7B00:
        if (cla_cond(cndl, s->nf, s->zf, s->tf, s->luf, s->lvf)) { uint32_t t = s->mr[la]; s->mr[la] = s->mr[lb]; s->mr[lb] = t; }
        return 0;
    case 0x7B40: case 0x7B80: case 0x7BC0: {
        int n = ((lsw >> 2) & 0x1F) + 1; uint32_t v = s->mr[la];
        if (msw == 0x7B40) v = (uint32_t)((int64_t)(int32_t)v >> n);
        else if (msw == 0x7B80) v = (uint32_t)((uint64_t)v >> n);
        else v = (uint32_t)((uint64_t)v << n);
        s->mr[la] = v; zn_i(s, v); return 0; }
    case 0x7C00: s->mr[la] = cb(s, MRF(lb) * MRF(lc)); return 0;
    case 0x7C20: s->mr[la] = cb(s, MRF(lb) + MRF(lc)); return 0;
    case 0x7C40: s->mr[la] = cb(s, MRF(lb) - MRF(lc)); return 0;
    case 0x7C60: s->mr[la] = s->mr[lb] & s->mr[lc]; zn_i(s, s->mr[la]); return 0;
    case 0x7C80: s->mr[la] = s->mr[lb] | s->mr[lc]; zn_i(s, s->mr[la]); return 0;
    case 0x7CA0: s->mr[la] = s->mr[lb] ^ s->mr[lc]; zn_i(s, s->mr[la]); return 0;
    case 0x7CC0: s->mr[la] = s->mr[lb] + s->mr[lc]; zn_i(s, s->mr[la]); return 0;
    case 0x7CE0: s->mr[la] = s->mr[lb] - s->mr[lc]; zn_i(s, s->mr[la]); return 0;
    case 0x7D00: case 0x7D20: case 0x7D40: {
        float x = MRF(la), y = MRF(lb);
        s->zf = x == y; s->nf = x < y;
        if ((msw == 0x7D20 && x < y) || (msw == 0x7D40 && x > y)) s->mr[la] = s->mr[lb];
        return 0; }
    case 0x7F20: { int32_t x = (int32_t)s->mr[la], y = (int32_t)s->mr[lb]; s->zf = x == y; s->nf = x < y; return 0; }
    case 0x7D60: s->mr[la] = sat_i(MRF(lb), -2147483648.0, 2147483647.0, 0); return 0;
    case 0x7DA0: s->mr[la] = sat_i(MRF(lb), 0.0, 4294967295.0, 0); return 0;
    case 0x7DE0: s->mr[la] = sat_i(MRF(lb), -32768.0, 32767.0, 0); return 0;
    case 0x7E60: s->mr[la] = sat_i(MRF(lb), -32768.0, 32767.0, 1); return 0;
    case 0x7EA0: s->mr[la] = sat_i(MRF(lb), 0.0, 65535.0, 0); return 0;
    case 0x7EC0: s->mr[la] = sat_i(MRF(lb), 0.0, 65535.0, 1); return 0;
    case 0x7D80: s->mr[la] = cb(s, (float)(int32_t)s->mr[lb]); return 0;
    case 0x7DC0: s->mr[la] = cb(s, (float)s->mr[lb]); return 0;
    case 0x7E80: s->mr[la] = cb(s, (float)(int16_t)(s->mr[lb] & M16)); return 0;
    case 0x7EE0: s->mr[la] = cb(s, (float)(s->mr[lb] & M16)); return 0;
    case 0x7E00: s->mr[la] = cb(s, fmodf(MRF(lb), 1.0f)); return 0;
    case 0x7E20: { uint32_t v = s->mr[lb] & 0x7FFFFFFFu; s->mr[la] = v; zn_f(s, v); return 0; }
    case 0x7E40: { float x = MRF(lb);
        if (x == 0) { s->lvf = 1; s->mr[la] = 0x7F7FFFFFu; } else s->mr[la] = cb(s, 1.0f / sqrtf(fabsf(x))); return 0; }
    case 0x7F00: { float x = MRF(lb);
        if (x == 0) { s->lvf = 1; s->mr[la] = 0x7F7FFFFFu | (s->mr[lb] & 0x80000000u); } else s->mr[la] = cb(s, 1.0f / x); return 0; }
    case 0x7F40: s->tf = cla_cond((int)(lsw & 0xF), s->nf, s->zf, s->tf, s->luf, s->lvf); return 0;
    case 0x7F80: return 1;
    case 0x7F60: case 0x7FA0: case 0x7F90: case 0x7FB0: return 0;
    }
    if ((msw & 0xFFC0) == 0x7FC0) {
        int r = (msw >> 5) & 1;
        *mar_reg = r;
        *mar_val = (msw & 0x10) ? (s->mr[msw & 3] + lsw) & 0xFFFF : lsw;
        return 0;
    }
    return -1;
}

static int get_u32_list(PyObject *o, uint32_t *out, int n) {
    for (int i = 0; i < n; i++) {
        PyObject *it = PySequence_GetItem(o, i);
        if (!it) return -1;
        out[i] = (uint32_t)PyLong_AsUnsignedLongMask(it);
        Py_DECREF(it);
    }
    return PyErr_Occurred() ? -1 : 0;
}

/* cla_state(core) -> (mr, mar, flags, mrpc); cla_state(core, mr, mar, flags, mrpc) sets it */
static PyObject *cla_state(PyObject *self, PyObject *args) {
    PyObject *core, *mr = NULL, *mar = NULL, *fl = NULL;
    unsigned long mrpc = 0;
    if (!PyArg_ParseTuple(args, "O!|OOOk", &CoreType, &core, &mr, &mar, &fl, &mrpc)) return NULL;
    Cla *s = &((Core *)core)->cla;
    if (mr) {
        uint32_t f6[6];
        if (get_u32_list(mr, s->mr, 4) < 0 || get_u32_list(mar, s->mar, 2) < 0 || get_u32_list(fl, f6, 6) < 0) return NULL;
        s->nf = f6[0]; s->zf = f6[1]; s->tf = f6[2]; s->luf = f6[3]; s->lvf = f6[4]; s->rndf32 = f6[5];
        s->mrpc = (uint32_t)mrpc;
        Py_RETURN_NONE;
    }
    return Py_BuildValue("[kkkk][kk][iiiiii]k", (unsigned long)s->mr[0], (unsigned long)s->mr[1], (unsigned long)s->mr[2],
                         (unsigned long)s->mr[3], (unsigned long)s->mar[0], (unsigned long)s->mar[1], s->nf, s->zf, s->tf,
                         s->luf, s->lvf, s->rndf32, (unsigned long)s->mrpc);
}

/* run a task from entry to MSTOP at the core's current time; 0, or -1 on an error */
static int cla_exec_(Core *c, uint32_t entry, long *steps_out, long *unknown_out);

/* the CLA's reads and writes are never part of a CPU loop's pass */
static int cla_exec(Core *c, uint32_t entry, long *steps_out, long *unknown_out) {
    int was = c->spin ? c->spin->event : 0;
    if (c->spin) c->spin->event = 1;
    int r = cla_exec_(c, entry, steps_out, unknown_out);
    if (c->spin) c->spin->event = was;
    return r;
}

static int cla_exec_(Core *c, uint32_t entry, long *steps_out, long *unknown_out) {
    Cla *sp = &c->cla;
#define s (*sp)
    uint32_t pc = entry;
    /* the flags of the last four instructions, packed (a branch decides on the oldest) */
#define CLA_FL ((uint8_t)(s.nf | s.zf << 1 | s.tf << 2 | s.luf << 3 | s.lvf << 4))
    uint8_t hist[4], fl = CLA_FL;      /* fl: the flags now (MNOP leaves them) */
    for (int i = 0; i < 4; i++) hist[i] = fl;
    int hp = 0;
    int pend_n[2] = {0, 0}; uint32_t pend_v[2] = {0, 0}; int pend_r[2] = {-1, -1}; int npend = 0;
    int br_left = 0; uint32_t br_tgt = 0;
    long steps = 0, unknown = -1;
    while (steps < 200000) {
        steps++;
        for (int i = 0; i < npend; i++) {
            if (--pend_n[i] <= 0) {
                s.mar[pend_r[i]] = pend_v[i];
                pend_n[i] = pend_n[npend - 1]; pend_v[i] = pend_v[npend - 1]; pend_r[i] = pend_r[npend - 1];
                npend--; i--;
            }
        }
        uint32_t lsw = c->mem[pc & M22], msw = c->mem[(pc + 1) & M22];
        hist[hp] = fl;
        hp = (hp + 1) & 3;
        uint8_t old = hist[hp];
        int mreg = -1; uint32_t mval = 0;
        int r = 0;
        if (msw != 0x7FA0) {           /* MNOP (the pipeline padding: much of a task) does nothing */
            r = cla_step(c, sp, pc, msw, lsw, &mreg, &mval);
            if (c->err) return -1;
            fl = CLA_FL;
        }
        if (r == -1) { unknown = (long)pc; break; }
        if (mreg >= 0 && npend < 2) { pend_r[npend] = mreg; pend_v[npend] = mval; pend_n[npend] = 3; npend++; }
        if (r == 1) {
            for (int i = 0; i < npend; i++) s.mar[pend_r[i]] = pend_v[i];
            break;
        }
        if (r == 2) {
            int cnd = msw & 0xF;
            if (cla_cond(cnd, old & 1, old >> 1 & 1, old >> 2 & 1, old >> 3 & 1, old >> 4 & 1)) {
                if ((msw & 0xFFF0) == 0x79A0) br_tgt = 0x9000 + s.mrpc;
                else {
                    int32_t off = (int16_t)lsw;
                    br_tgt = (uint32_t)(((int32_t)pc + off - 0x9000) & 0xFFF) + 0x9000;
                    if ((msw & 0xFFF0) == 0x7990) s.mrpc = (pc + 8 - 0x9000) & 0xFFF;
                }
                br_left = 4;
            }
        }
        if (br_left) {
            if (--br_left == 0) { pc = br_tgt; continue; }
        }
        pc += 2;
    }
#undef CLA_FL
#undef s
    *steps_out = steps;
    *unknown_out = unknown;
    return 0;
}

/* cla_run(core, entry) -> (steps, unknown_pc or -1); the CLA registers live in the core */
static PyObject *cla_run(PyObject *self, PyObject *args) {
    PyObject *core;
    unsigned long entry;
    if (!PyArg_ParseTuple(args, "O!k", &CoreType, &core, &entry)) return NULL;
    Core *c = (Core *)core;
    c->err = 0;
    PyObject *cv = PyObject_GetAttr(c->clock, s_cycles);
    if (!cv) return NULL;
    c->cycles = PyLong_AsLongLong(cv);
    Py_DECREF(cv);
    long steps = 0, unknown = -1;
    if (cla_exec(c, (uint32_t)entry, &steps, &unknown) < 0) return NULL;
    return Py_BuildValue("ll", steps, unknown);
}
