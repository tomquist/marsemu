/* Native F2803x PIE, ePWM1-7, ADC and CLA task dispatch.
 *
 * A mirror of the Python models (f2803x_sys.Pie, f2803x_pwm.Epwm,
 * f2803x_adc.Adc, cla.Cla's triggers), which stay the reference: each
 * function here is a line-by-line port of its Python namesake.  It exists
 * because the control loops of the C2000 firmware touch these peripherals
 * tens of thousands of times per emulated second; done in Python, every
 * access and every timed event would leave the native core.
 *
 * Timed events (ePWM turning points, ADC conversions and their EOCs, the CLA's
 * end of task) are kept here and run by the core's loop at the first
 * instruction boundary at or after their time, the way the machine's Python
 * loop runs clock events.  The Python side (f2803x_native.py) switches the
 * peripheral objects to facades that read and write this state.
 */

#define FP_PIE 1
#define FP_ADC 2
#define FP_CLA 3
#define FP_EPWM 4          /* FP_EPWM + k - 1 for ePWMk, k = 1..7 */
#define FP_NONE_T INT64_MAX

enum { EP_UP = 0, EP_DOWN = 1, EP_UPDOWN = 2, EP_FREEZE = 3 };
enum { K_ZERO = 0, K_PRD = 1 };

typedef struct {
    uint32_t ier[13], ifr[13], ack, ctrl;
    uint64_t counts[13][9];
    uint16_t regs[0x20];
} NPie;

typedef struct {
    int idx;
    uint16_t regs[0x40];
    int64_t prd, prd_shadow, cmpa, cmpa_sh, cmpb, cmpb_sh;
    int prd_pend, cmpa_pend, cmpb_pend;
    int64_t anchor_t, anchor_ctr;
    int dir;
    int et_int_cnt, et_soca_cnt, et_socb_cnt;
    uint32_t etflg, tzflg;
    int tz_forced;
    int64_t periods, ints, socas;
    int64_t ev_t;               /* next turning-point event, FP_NONE_T = none */
    int ev_kind;
    int sync_valid;
    int64_t sync_mprd, sync_prd;
    uint32_t sync_phs, sync_ctl;
    int py_boundary, py_tz;     /* Python listeners to call */
} NEpwm;

typedef struct {
    int64_t t;
    int n;
    uint8_t order[16];
} NEoc;

typedef struct {
    uint16_t regs[0x80];
    uint16_t socctl[16];
    uint16_t result[16];
    uint32_t pending, intflg, intovf;
    int rr;
    int64_t busy_until, conv_t;
    NEoc eoc[8];
    int neoc;
    uint64_t conversions;
    uint32_t temp_raw;
    int maps_valid;
    uint32_t trig_map[32];      /* TRIGSEL -> SOCs */
    uint16_t eoc_ints[16];      /* EOC i -> ADCINTn bits (bit n) */
    uint16_t cont;              /* ADCINTn continuous (bit n) */
    uint32_t starts[3];         /* ADCINT1/2 -> SOCs they start */
    int starts_valid;
    PyObject *sample;           /* fn(channel mask) -> 16 values, or NULL: all 0 */
    PyObject *listeners;        /* fn(n) for Python ADCINT listeners, or NULL */
} NAdc;

typedef struct {
    uint16_t regs[0x80];
    uint32_t mifr, miovf, mier, mirun;
    int running;
    uint64_t task_counts[9], task_cycles[9];
    int64_t end_t;
    uint64_t faults;
    int64_t last_unknown;
} NCla;

struct Fp {
    NPie pie;
    NEpwm ep[8];
    NAdc adc;
    NCla cla;
    uint8_t route[0x200];       /* IO page (a >> 6, a < 0x8000) -> FP_* (0: Python) */
    int64_t next;
    int trace;                  /* tracers: IO goes through the Python objects */
    PyObject *boundary_cb;      /* fn(k, kind) */
    PyObject *tz_cb;            /* fn(k) */
    uint64_t exc_count, int_count[32];
};

static int fp_tracing(Core *c) { return c->fp && c->fp->trace; }

static int64_t pymod(int64_t a, int64_t b) { int64_t r = a % b; return r < 0 ? r + b : r; }
static int64_t pydiv(int64_t a, int64_t b) { int64_t q = a / b; if ((a % b) && ((a < 0) != (b < 0))) q--; return q; }

static void fp_recalc(Core *c) {
    Fp *f = c->fp;
    int64_t n = FP_NONE_T;
    for (int k = 1; k <= 7; k++) if (f->ep[k].ev_t < n) n = f->ep[k].ev_t;
    if (f->adc.conv_t < n) n = f->adc.conv_t;
    for (int i = 0; i < f->adc.neoc; i++) if (f->adc.eoc[i].t < n) n = f->adc.eoc[i].t;
    if (f->cla.end_t < n) n = f->cla.end_t;
    f->next = n;
}

/* ===================================================================== PIE */
static void core_interrupt(Core *c, int k) {
    uint32_t bit = 1u << (k - 1);
    c->ifr |= bit;
    if (c->idle_wait && (c->ier & bit)) c->idle_wait = 0;
}

static void pie_update(Core *c, int g) {
    NPie *p = &c->fp->pie;
    if ((p->ifr[g] & p->ier[g]) && !((p->ack >> (g - 1)) & 1)) {
        p->ack |= 1u << (g - 1);
        core_interrupt(c, g);
    }
}

static void pie_raise(Core *c, int g, int i) {
    c->fp->pie.ifr[g] |= 1u << (i - 1);
    pie_update(c, g);
}

static uint32_t pie_vector(Core *c, int k) {
    NPie *p = &c->fp->pie;
    if (!(p->ctrl & 1)) return rd32(c, 0x3FFFC0 + 2 * k);
    if (k >= 1 && k <= 12) {
        uint32_t pend = p->ifr[k] & p->ier[k];
        if (pend) {
            int y = __builtin_ctz(pend) + 1;
            p->ifr[k] &= ~(1u << (y - 1));
            p->counts[k][y]++;
            uint32_t a = 0xD40 + ((k - 1) * 8 + y - 1) * 2;
            p->ctrl = (p->ctrl & 1) | (a & 0xFFFE);
            return rd32(c, a);
        }
        return rd32(c, 0xD40 + (k - 1) * 16);
    }
    return rd32(c, 0xD00 + 2 * k);
}

static uint32_t pie_read(Core *c, uint32_t off) {
    NPie *p = &c->fp->pie;
    if (off == 0) return p->ctrl & M16;
    if (off == 1) return p->ack;
    uint32_t g = off / 2;
    if (g >= 1 && g <= 12) return off % 2 == 0 ? p->ier[g] : p->ifr[g];
    return off < 0x20 ? p->regs[off] : 0;
}

static void pie_write(Core *c, uint32_t off, uint32_t v) {
    NPie *p = &c->fp->pie;
    if (off == 0) { p->ctrl = (p->ctrl & 0xFFFE) | (v & 1); return; }
    if (off == 1) {
        p->ack &= ~v;
        for (int g = 1; g <= 12; g++) if ((v >> (g - 1)) & 1) pie_update(c, g);
        return;
    }
    uint32_t g = off / 2;
    if (g >= 1 && g <= 12) {
        if (off % 2 == 0) p->ier[g] = v & 0xFF; else p->ifr[g] = v & 0xFF;
        pie_update(c, (int)g);
        return;
    }
    if (off < 0x20) p->regs[off] = (uint16_t)v;
}

/* ===================================================================== CLA */
static int cla_exec(Core *c, uint32_t entry, long *steps_out, long *unknown_out);
static void cla_dispatch(Core *c);

static void cla_force(Core *c, uint32_t bits) {
    NCla *a = &c->fp->cla;
    uint32_t ov = a->mifr & bits;
    a->miovf |= ov;
    a->mifr |= bits & 0xFF;
    cla_dispatch(c);
}

static void cla_iack(Core *c, uint32_t v) {
    if (c->fp->cla.regs[0x10] & 0x4) cla_force(c, v & 0xFF);
}

static void cla_peripheral_irq(Core *c, int adc, int n) {
    /* adc: 1 = ADCINTn, 0 = ePWMn INT, -1 = TINT0 */
    NCla *a = &c->fp->cla;
    if (!(n >= 1 && n <= 8) && adc != -1) return;
    uint32_t sel = a->regs[0x14] | ((uint32_t)a->regs[0x15] << 16);
    if (adc == -1) {
        if (((sel >> 28) & 0xF) == 2) cla_force(c, 1u << 7);
        return;
    }
    uint32_t s = (sel >> (4 * (n - 1))) & 0xF;
    if ((adc == 1 && s == 0) || (adc == 0 && s == 2 && n <= 7)) cla_force(c, 1u << (n - 1));
}

static void cla_task_end(Core *c) {
    NCla *a = &c->fp->cla;
    a->end_t = FP_NONE_T;
    int task = a->running;
    a->running = 0;
    a->mirun = 0;
    if (task) pie_raise(c, 11, task);
    cla_dispatch(c);
    fp_recalc(c);
}

static void cla_dispatch(Core *c) {
    NCla *a = &c->fp->cla;
    if (a->running) return;
    uint32_t ready = a->mifr & a->mier;
    if (!ready) return;
    uint32_t bit = ready & (~ready + 1);
    int task = __builtin_ctz(bit) + 1;
    a->mifr &= ~bit;
    a->mirun = bit;
    a->running = task;
    long steps = 0, unknown = -1;
    uint32_t entry = 0x9000 + (a->regs[task - 1] & 0xFFF);
    if (cla_exec(c, entry, &steps, &unknown) < 0) return;
    if (unknown >= 0) { a->faults++; a->last_unknown = unknown; }
    a->task_counts[task]++;
    a->task_cycles[task] += (uint64_t)steps;
    if (!((c->fp->pie.ier[11] >> (task - 1)) & 1) && !(a->mifr & a->mier)) {
        cla_task_end(c);
        return;
    }
    a->end_t = c->cycles + (steps + 4 > 1 ? steps + 4 : 1);
    fp_recalc(c);
}

static uint32_t cla_read(Core *c, uint32_t off) {
    NCla *a = &c->fp->cla;
    Cla *s = &c->cla;
    if (off == 0x20) return a->mifr;
    if (off == 0x21) return a->miovf;
    if (off == 0x25) return a->mier;
    if (off == 0x26) return a->mirun;
    if (off == 0x28) return 0;
    if (off == 0x2A || off == 0x2B) return s->mar[off - 0x2A] & M16;
    if (off == 0x2E || off == 0x2F) {
        uint32_t v = (uint32_t)(s->lvf | s->luf << 1 | s->nf << 2 | s->zf << 3 | s->tf << 6 | s->rndf32 << 9)
                     | ((s->mrpc & 0xFFF) << 12);
        return off & 1 ? v >> 16 : v & M16;
    }
    if (off >= 0x30 && off < 0x40) {
        uint32_t i = (off - 0x30) / 4, r = (off - 0x30) % 4;
        if (r < 2) return r ? (s->mr[i] >> 16) & M16 : s->mr[i] & M16;
    }
    if (off == 0x22 || off == 0x23 || off == 0x24) return 0;
    return off < 0x80 ? a->regs[off] : 0;
}

static void cla_write(Core *c, uint32_t off, uint32_t v) {
    NCla *a = &c->fp->cla;
    if (off == 0x22) { cla_force(c, v & 0xFF); return; }
    if (off == 0x23) { a->mifr &= ~v; return; }
    if (off == 0x24) { a->miovf &= ~v; return; }
    if (off == 0x25) { a->mier = v & 0xFF; cla_dispatch(c); return; }
    if (off == 0x10) {
        if (v & 3) { a->mier = 0; if (v & 1) a->mifr = a->miovf = 0; }
        a->regs[0x10] = v & 0x4;
        return;
    }
    if (off < 8) v &= 0xFFF;
    if (off < 0x80) a->regs[off] = (uint16_t)v;
}

/* ===================================================================== ADC */
static void adc_convert(Core *c);
static int adc_flag(Core *c, uint32_t bits);

static void adc_maps(NAdc *d) {
    if (d->maps_valid) return;
    memset(d->trig_map, 0, sizeof d->trig_map);
    for (int i = 0; i < 16; i++) d->trig_map[(d->socctl[i] >> 11) & 31] |= 1u << i;
    memset(d->eoc_ints, 0, sizeof d->eoc_ints);
    d->cont = 0;
    for (int n = 1; n < 10; n++) {
        uint32_t isel = d->regs[0x8 + (n - 1) / 2];
        uint32_t byte = n % 2 == 0 ? (isel >> 8) & 0xFF : isel & 0xFF;
        if (byte & 0x20) {
            if ((byte & 0x1F) < 16) d->eoc_ints[byte & 0x1F] |= (uint16_t)(1u << n);
            if (byte & 0x40) d->cont |= (uint16_t)(1u << n);
        }
    }
    d->maps_valid = 1;
}

static uint32_t *adc_starts(NAdc *d) {
    if (!d->starts_valid) {
        uint32_t socsel = d->regs[0x14] | ((uint32_t)d->regs[0x15] << 16);
        d->starts[0] = d->starts[1] = d->starts[2] = 0;
        for (int j = 0; j < 16; j++) {
            uint32_t k = (socsel >> (2 * j)) & 3;
            if (k == 1 || k == 2) d->starts[k] |= 1u << j;
        }
        d->starts_valid = 1;
    }
    return d->starts;
}

static int adc_trigger(Core *c, int src) {
    NAdc *d = &c->fp->adc;
    adc_maps(d);
    uint32_t hit = (src >= 0 && src < 32) ? d->trig_map[src] : 0;
    if (hit) return adc_flag(c, hit);
    return 0;
}

static int adc_flag(Core *c, uint32_t bits) {
    NAdc *d = &c->fp->adc;
    d->pending |= bits;
    if (d->conv_t == FP_NONE_T) {
        if (d->busy_until <= c->cycles) adc_convert(c);
        else { d->conv_t = d->busy_until; fp_recalc(c); }
    }
    return c->err ? -1 : 0;
}

static void adc_convert(Core *c) {
    NAdc *d = &c->fp->adc;
    d->conv_t = FP_NONE_T;
    int64_t t = 0;
    int div = (d->regs[1] & 1) ? 2 : 1;
    uint32_t ctl1 = d->regs[0];
    int32_t off = d->regs[0x41] & 0x1FF;
    if (off & 0x100) off -= 0x200;
    uint32_t pend = d->pending;
    /* the analog values: one call to the board model for the batch */
    uint32_t vals[16] = {0};
    uint32_t need = 0;
    for (int i = 0; i < 16; i++) {
        if (!((pend >> i) & 1)) continue;
        uint32_t ch = (d->socctl[i] >> 6) & 0xF;
        if (ch == 13 && (ctl1 & 2)) continue;
        if (ch == 5 && (ctl1 & 1)) continue;
        need |= 1u << ch;
    }
    if (need && d->sample) {
        /* the board model only reads: cached IO reads stay valid */
        uint32_t ep = c->epoch;
        sync_clock(c);
        c->epoch = ep;
        PyObject *r = PyObject_CallFunction(d->sample, "k", (unsigned long)need);
        if (!r) { c->err = 1; return; }
        PyObject *seq = PySequence_Fast(r, "sample() returns 16 values");
        Py_DECREF(r);
        if (!seq) { c->err = 1; return; }
        Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
        for (Py_ssize_t i = 0; i < n && i < 16; i++) {
            long v = PyLong_AsLong(PySequence_Fast_GET_ITEM(seq, i));
            vals[i] = (uint32_t)v;
            if (v == -1 && PyErr_Occurred()) { Py_DECREF(seq); c->err = 1; return; }
            vals[i] = (uint32_t)(int32_t)v;
        }
        Py_DECREF(seq);
    }
    NEoc e;
    e.n = 0;
    int start = d->rr + 1;
    for (int k = 0; k < 16; k++) {
        int i = (start + k) & 15;
        if (!((pend >> i) & 1)) continue;
        uint32_t ctl = d->socctl[i];
        uint32_t ch = (ctl >> 6) & 0xF;
        int32_t v;
        if (ch == 13 && (ctl1 & 2)) v = 0;
        else if (ch == 5 && (ctl1 & 1)) v = (int32_t)d->temp_raw;
        else v = (int32_t)vals[ch];
        v += off;
        v = v < 0 ? 0 : (v > 4095 ? 4095 : v);
        d->result[i] = (uint16_t)v;
        c->mem[0x0B00 + i] = (uint16_t)v;
        d->rr = i;
        t += ((ctl & 0x3F) + 14) * div;
        e.order[e.n++] = (uint8_t)i;
    }
    d->conversions += (uint64_t)e.n;
    d->pending = 0;
    if (!e.n) return;
    d->busy_until = c->cycles + (t > 1 ? t : 1);
    e.t = d->busy_until;
    if (d->neoc < 8) d->eoc[d->neoc++] = e;
    fp_recalc(c);
}

static const int ADC_PIE[10][2] = {{0, 0}, {1, 1}, {1, 2}, {10, 3}, {10, 4}, {10, 5}, {10, 6}, {10, 7}, {10, 8}, {1, 6}};

static int adc_eoc(Core *c, NEoc *e) {
    NAdc *d = &c->fp->adc;
    adc_maps(d);
    for (int j = 0; j < e->n; j++) {
        int i = e->order[j];
        uint32_t ints = d->eoc_ints[i];
        for (int n = 1; n < 10; n++) {
            if (!((ints >> n) & 1)) continue;
            uint32_t bit = 1u << (n - 1);
            if (d->intflg & bit) {
                d->intovf |= bit;
                if (!((d->cont >> n) & 1)) continue;
            }
            d->intflg |= bit;
            pie_raise(c, ADC_PIE[n][0], ADC_PIE[n][1]);
            if (n == 1 || n == 2) pie_raise(c, 10, n);
            cla_peripheral_irq(c, 1, n);
            if (c->err) return -1;
            if (d->listeners) {
                sync_clock(c);
                PyObject *r = PyObject_CallFunction(d->listeners, "i", n);
                if (!r) { c->err = 1; return -1; }
                Py_DECREF(r);
            }
            if (n == 1 || n == 2) {
                uint32_t st = adc_starts(d)[n];
                if (st && adc_flag(c, st) < 0) return -1;
            }
        }
    }
    return 0;
}

static uint32_t adc_read(Core *c, uint32_t off) {
    NAdc *d = &c->fp->adc;
    if (off == 0x0) return (d->regs[0] & 0x40FF) | 0x4000;
    if (off == 0x4) return d->intflg;
    if (off == 0x6) return d->intovf;
    if (off == 0x18) return d->pending;
    if (off >= 0x20 && off < 0x30) return d->socctl[off - 0x20];
    if (off == 0x5 || off == 0x7 || off == 0x1A || off == 0x1E) return 0;
    return off < 0x80 ? d->regs[off] : 0;
}

static void adc_reset(Core *c) {
    NAdc *d = &c->fp->adc;
    memset(d->regs, 0, sizeof d->regs);
    d->maps_valid = 0;
    d->starts_valid = 0;
    memset(d->result, 0, sizeof d->result);
    for (int i = 0; i < 16; i++) c->mem[0x0B00 + i] = 0;
    memset(d->socctl, 0, sizeof d->socctl);
    d->pending = d->intflg = d->intovf = 0;
    d->rr = 15;
    d->regs[0x0] = 0x4000;
    d->regs[0x4F] = 0x0003;
    d->conv_t = FP_NONE_T;
}

static int adc_write(Core *c, uint32_t off, uint32_t v) {
    NAdc *d = &c->fp->adc;
    if (off == 0x0) {
        if (v & 0x8000) { adc_reset(c); fp_recalc(c); return 0; }
        d->regs[0] = (uint16_t)(v & 0x40FF);
        return 0;
    }
    if (off == 0x5) { d->intflg &= ~v; return 0; }
    if (off == 0x7) { d->intovf &= ~v; return 0; }
    if (off == 0x1A) { if (v) return adc_flag(c, v); return 0; }
    if (off == 0x1E) return 0;
    if (off >= 0x20 && off < 0x30) { d->socctl[off - 0x20] = (uint16_t)v; d->maps_valid = 0; return 0; }
    if (off >= 0x8 && off <= 0xC) d->maps_valid = 0;
    if (off == 0x14 || off == 0x15) d->starts_valid = 0;
    if (off < 0x80) d->regs[off] = (uint16_t)v;
    return 0;
}

/* ==================================================================== ePWM */
static inline NEpwm *EP(Core *c, int k) { return &c->fp->ep[k]; }

static int ep_div(NEpwm *e) {
    uint32_t t = e->regs[0x00];
    uint32_t hsp = (t >> 7) & 7, clk = (t >> 10) & 7;
    return (int)((hsp ? 2 * hsp : 1) * (1u << clk));
}

static inline int ep_mode(NEpwm *e) { return e->regs[0x00] & 3; }

static int64_t ep_period_cycles(NEpwm *e) {
    int div = ep_div(e);
    if (ep_mode(e) == EP_UPDOWN) return 2 * e->prd * div;
    return (e->prd + 1) * div;
}

static void ep_next_turn(NEpwm *e, int64_t *t, int *kind) {
    int mode = ep_mode(e);
    int64_t prd = e->prd, c = e->anchor_ctr, dt;
    if (mode == EP_FREEZE || prd == 0) { *t = -1; return; }
    if (mode == EP_UP) {
        if (c >= prd) { dt = prd + 1 - c; *kind = K_ZERO; } else { dt = prd - c; *kind = K_PRD; }
    } else if (mode == EP_DOWN) {
        if (c == 0) { dt = 1; *kind = K_PRD; } else { dt = c; *kind = K_ZERO; }
    } else {
        if (e->dir > 0) { dt = prd - c; *kind = K_PRD; } else { dt = c; *kind = K_ZERO; }
    }
    if (dt <= 0) {
        dt = mode != EP_UPDOWN ? 1 : 2 * prd;
        *kind = *kind == K_PRD ? K_ZERO : K_PRD;
    }
    *t = e->anchor_t + dt * ep_div(e);
}

static void ep_apply_loads(NEpwm *e, int kind) {
    uint32_t cmpctl = e->regs[0x07];
    uint32_t lda = cmpctl & 3, ldb = (cmpctl >> 2) & 3;
    int zero = kind == K_ZERO;
    if (e->cmpa_pend && (lda == 2 || (lda == 0 && zero) || (lda == 1 && !zero))) { e->cmpa = e->cmpa_sh; e->cmpa_pend = 0; }
    if (e->cmpb_pend && (ldb == 2 || (ldb == 0 && zero) || (ldb == 1 && !zero))) { e->cmpb = e->cmpb_sh; e->cmpb_pend = 0; }
    if (e->prd_pend && zero) { e->prd = e->prd_shadow; e->prd_pend = 0; }
}

static void ep_turn(NEpwm *e, int64_t t, int kind) {
    e->anchor_t = t;
    if (kind == K_ZERO) { e->anchor_ctr = 0; e->dir = 1; e->periods++; }
    else { e->anchor_ctr = e->prd; if (ep_mode(e) == EP_UPDOWN) e->dir = -1; }
    ep_apply_loads(e, kind);
}

static void ep_catch_up(Core *c, NEpwm *e) {
    if (e->ev_t != FP_NONE_T || !(e->cmpa_pend || e->cmpb_pend || e->prd_pend)) return;
    int64_t now = c->cycles;
    for (int i = 0; i < 8; i++) {
        int64_t t; int kind = 0;
        ep_next_turn(e, &t, &kind);
        if (t < 0 || t > now) return;
        ep_turn(e, t, kind);
        if (!(e->cmpa_pend || e->cmpb_pend || e->prd_pend)) return;
    }
    ep_apply_loads(e, K_ZERO);
    ep_apply_loads(e, K_PRD);
}

static int64_t ep_counter(Core *c, NEpwm *e) {
    ep_catch_up(c, e);
    int mode = ep_mode(e);
    if (mode == EP_FREEZE) return e->anchor_ctr;
    int64_t ticks = pydiv(c->cycles - e->anchor_t, ep_div(e));
    int64_t prd = e->prd, ct = e->anchor_ctr;
    if (mode == EP_UP) return prd == 0 ? 0 : pymod(ct + ticks, prd + 1);
    if (mode == EP_DOWN) return prd == 0 ? 0 : pymod(ct - ticks, prd + 1);
    if (prd == 0) return 0;
    int64_t pos = pymod((e->dir > 0 ? ct : 2 * prd - ct) + ticks, 2 * prd);
    return pos <= prd ? pos : 2 * prd - pos;
}

static int ep_direction(Core *c, NEpwm *e) {
    int mode = ep_mode(e);
    if (mode == EP_UP) return 1;
    if (mode == EP_DOWN) return -1;
    if (mode == EP_FREEZE || e->prd == 0) return e->dir;
    int64_t ticks = pydiv(c->cycles - e->anchor_t, ep_div(e));
    int64_t pos = pymod((e->dir > 0 ? e->anchor_ctr : 2 * e->prd - e->anchor_ctr) + ticks, 2 * e->prd);
    return pos < e->prd ? 1 : -1;
}

static void ep_sync_key(NEpwm *e, int64_t *prd, uint32_t *phs, uint32_t *ctl) {
    *prd = ep_period_cycles(e);
    *phs = e->regs[0x03];
    *ctl = e->regs[0x00] & 0x2007;
}

static int ep_needs_events(Core *c, NEpwm *e) {
    if (e->py_boundary) return 1;
    uint32_t etsel = e->regs[0x19], etps = e->regs[0x1A];
    if (((etsel & 0x8) && (etps & 3)) || ((etsel & 0x800) && (etps & 0x300)) || ((etsel & 0x8000) && (etps & 0x3000)))
        return 1;
    if (((e->regs[0x00] >> 4) & 3) == 1) {
        for (int k = e->idx + 1; k <= 7; k++) {
            NEpwm *n = EP(c, k);
            if (n->regs[0x00] & 0x4) {
                int64_t p; uint32_t ph, ct;
                ep_sync_key(n, &p, &ph, &ct);
                if (!(n->sync_valid && n->sync_mprd == ep_period_cycles(e) && n->sync_prd == p && n->sync_phs == ph
                      && n->sync_ctl == ct))
                    return 1;
            }
            if (((n->regs[0x00] >> 4) & 3) != 0) break;
        }
    }
    return 0;
}

static void ep_schedule(Core *c, NEpwm *e) {
    e->ev_t = FP_NONE_T;
    int mode = ep_mode(e);
    if (mode != EP_FREEZE && e->prd != 0 && ep_needs_events(c, e)) {
        int64_t t; int kind = 0;
        ep_next_turn(e, &t, &kind);
        e->ev_t = t;
        e->ev_kind = kind;
    }
    fp_recalc(c);
}

static void ep_restart(Core *c, NEpwm *e, int64_t ctr, int dir) {
    e->anchor_t = c->cycles;
    e->anchor_ctr = ctr;
    e->dir = dir;
    ep_schedule(c, e);
}

static void ep_sync_in(Core *c, NEpwm *e) {
    uint32_t t = e->regs[0x00];
    if (!(t & 0x4)) return;
    ep_catch_up(c, e);
    int64_t phs = e->regs[0x03];
    int dir = 1;
    if (ep_mode(e) == EP_UPDOWN) dir = (t & 0x2000) ? 1 : -1;
    e->sync_mprd = e->idx > 1 ? ep_period_cycles(EP(c, e->idx - 1)) : 0;
    ep_sync_key(e, &e->sync_prd, &e->sync_phs, &e->sync_ctl);
    e->sync_valid = 1;
    ep_restart(c, e, e->prd ? (phs < e->prd ? phs : e->prd) : phs, dir);
}

static void ep_sync_out(Core *c, NEpwm *e) {
    for (int k = e->idx + 1; k <= 7; k++) {
        NEpwm *n = EP(c, k);
        ep_sync_in(c, n);
        if (((n->regs[0x00] >> 4) & 3) != 0) break;
    }
}

static inline int ep_et_match(uint32_t sel, int kind) {
    return (sel == 3) || (sel == 1 && kind == K_ZERO) || (sel == 2 && kind == K_PRD);
}

static int ep_event_trigger(Core *c, NEpwm *e, int kind) {
    uint32_t etsel = e->regs[0x19];
    if (!(etsel & 0x8908)) return 0;
    uint32_t etps = e->regs[0x1A];
    if ((etsel & 0x8) && ep_et_match(etsel & 7, kind)) {
        int prd = etps & 3;
        if (prd && ++e->et_int_cnt >= prd) {
            e->et_int_cnt = 0;
            if (!(e->etflg & 1)) {
                e->etflg |= 1;
                e->ints++;
                pie_raise(c, 3, e->idx);
                cla_peripheral_irq(c, 0, e->idx);
                if (c->err) return -1;
            }
        }
    }
    if ((etsel & 0x800) && ep_et_match((etsel >> 8) & 7, kind)) {
        int prd = (etps >> 8) & 3;
        if (prd && ++e->et_soca_cnt >= prd) {
            e->et_soca_cnt = 0;
            e->etflg |= 4;
            e->socas++;
            if (adc_trigger(c, 5 + 2 * (e->idx - 1)) < 0) return -1;
        }
    }
    if ((etsel & 0x8000) && ep_et_match((etsel >> 12) & 7, kind)) {
        int prd = (etps >> 12) & 3;
        if (prd && ++e->et_socb_cnt >= prd) {
            e->et_socb_cnt = 0;
            e->etflg |= 8;
            if (adc_trigger(c, 5 + 2 * (e->idx - 1) + 1) < 0) return -1;
        }
    }
    return 0;
}

static int ep_boundary(Core *c, NEpwm *e, int kind) {
    e->ev_t = FP_NONE_T;
    ep_turn(e, c->cycles, kind);
    if (ep_event_trigger(c, e, kind) < 0) return -1;
    if (kind == K_ZERO && ((e->regs[0x00] >> 4) & 3) == 1) ep_sync_out(c, e);
    if (e->py_boundary && c->fp->boundary_cb) {
        sync_clock(c);
        PyObject *r = PyObject_CallFunction(c->fp->boundary_cb, "is", e->idx, kind == K_ZERO ? "zero" : "prd");
        if (!r) { c->err = 1; return -1; }
        Py_DECREF(r);
    }
    ep_schedule(c, e);
    return 0;
}

static int ep_trip(Core *c, NEpwm *e, int oneshot) {
    e->tzflg |= 1 | (oneshot ? 4 : 2);
    e->tz_forced = 1;
    if (e->regs[0x15] & (oneshot ? 4 : 2)) pie_raise(c, 2, e->idx);
    if (e->py_tz && c->fp->tz_cb) {
        sync_clock(c);
        PyObject *r = PyObject_CallFunction(c->fp->tz_cb, "i", e->idx);
        if (!r) { c->err = 1; return -1; }
        Py_DECREF(r);
    }
    return 0;
}

static double ep_duty_calc(NEpwm *e, int64_t prd, int mode, uint32_t aq) {
    int zro = aq & 3, prd_a = (aq >> 2) & 3, cau = (aq >> 4) & 3, cad = (aq >> 6) & 3, cbu = (aq >> 8) & 3,
        cbd = (aq >> 10) & 3;
    int64_t ca = e->cmpa, cb = e->cmpb;
    int64_t et[8]; int ea[8]; int n = 0;
    int64_t T;
    if (mode == EP_UPDOWN) {
        T = 2 * prd;
        et[n] = 0; ea[n++] = zro;
        if (0 < ca && ca <= prd) { et[n] = ca; ea[n++] = cau; et[n] = T - ca; ea[n++] = cad; }
        if (0 < cb && cb <= prd) { et[n] = cb; ea[n++] = cbu; et[n] = T - cb; ea[n++] = cbd; }
        et[n] = prd; ea[n++] = prd_a;
    } else {
        T = prd + 1;
        et[n] = 0; ea[n++] = zro;
        if (ca <= prd) { et[n] = ca; ea[n++] = mode == EP_UP ? cau : cad; }
        if (cb <= prd) { et[n] = cb; ea[n++] = mode == EP_UP ? cbu : cbd; }
        et[n] = prd; ea[n++] = prd_a;
    }
    /* stable sort by time (Python's sort is stable) */
    for (int i = 1; i < n; i++) {
        int64_t tt = et[i]; int aa = ea[i]; int j = i - 1;
        while (j >= 0 && et[j] > tt) { et[j + 1] = et[j]; ea[j + 1] = ea[j]; j--; }
        et[j + 1] = tt; ea[j + 1] = aa;
    }
    int level = 0;
    int64_t hi = 0;
    for (int pass = 0; pass < 2; pass++) {
        hi = 0;
        int64_t last = 0;
        for (int i = 0; i < n; i++) {
            if (level) hi += et[i] - last;
            last = et[i];
            if (ea[i] == 1) level = 0; else if (ea[i] == 2) level = 1; else if (ea[i] == 3) level ^= 1;
        }
        if (level) hi += T - last;
    }
    return (double)hi / (double)T;
}

static double ep_duty(Core *c, NEpwm *e, int b) {
    ep_catch_up(c, e);
    if (e->tz_forced) {
        uint32_t act = (e->regs[0x14] >> (b ? 2 : 0)) & 3;
        return act == 1 ? 1.0 : 0.0;
    }
    uint32_t cs = (e->regs[0x0E] >> (b ? 2 : 0)) & 3;
    if (cs == 1) return 0.0;
    if (cs == 2) return 1.0;
    int mode = ep_mode(e);
    if (e->prd == 0 || mode == EP_FREEZE) return 0.0;
    return ep_duty_calc(e, e->prd, mode, e->regs[b ? 0x0C : 0x0B]);
}

static uint32_t ep_read(Core *c, NEpwm *e, uint32_t off) {
    if (off == 0x40) return (uint32_t)e->tz_forced;    /* not a register: the facade's outputs_enabled() */
    ep_catch_up(c, e);
    switch (off) {
    case 0x04: return (uint32_t)ep_counter(c, e) & M16;
    case 0x01: return (e->regs[0x01] & 0x06) | (ep_direction(c, e) > 0 ? 1 : 0);
    case 0x05: return (uint32_t)e->prd & M16;
    case 0x09: return (uint32_t)e->cmpa & M16;
    case 0x0A: return (uint32_t)e->cmpb & M16;
    case 0x07: return (e->regs[0x07] & 0x5F) | (e->cmpa_pend ? 0x100 : 0) | (e->cmpb_pend ? 0x200 : 0);
    case 0x1B: return e->etflg;
    case 0x16: return e->tzflg;
    case 0x1A: return (e->regs[0x1A] & 0x3333) | (e->et_int_cnt & 3) << 2 | (e->et_soca_cnt & 3) << 10
                      | (e->et_socb_cnt & 3) << 14;
    case 0x1C: case 0x1D: case 0x17: case 0x18: return 0;
    }
    return off < 0x40 ? e->regs[off] : 0;
}

static void ep_rearm(Core *c, NEpwm *e) {
    if (e->ev_t == FP_NONE_T && ep_mode(e) != EP_FREEZE && e->prd && ep_needs_events(c, e))
        ep_restart(c, e, ep_counter(c, e), ep_direction(c, e));
}

static int ep_write_inner(Core *c, NEpwm *e, uint32_t off, uint32_t v) {
    switch (off) {
    case 0x05:
        if (e->regs[0x00] & 0x8) {
            e->prd = v;
            int64_t ct = ep_counter(c, e);
            ep_restart(c, e, ct < (int64_t)v ? ct : (int64_t)v, ep_direction(c, e));
        } else {
            if (e->prd == 0 || ep_mode(e) == EP_FREEZE) { e->prd = v; ep_schedule(c, e); }
            e->prd_shadow = v;
            e->prd_pend = 1;
        }
        return 0;
    case 0x09:
        if (e->regs[0x07] & 0x10) e->cmpa = v; else { e->cmpa_sh = v; e->cmpa_pend = 1; }
        return 0;
    case 0x0A:
        if (e->regs[0x07] & 0x40) e->cmpb = v; else { e->cmpb_sh = v; e->cmpb_pend = 1; }
        return 0;
    case 0x04:
        ep_restart(c, e, v, ep_direction(c, e));
        return 0;
    case 0x00: {
        int64_t ct = ep_counter(c, e);
        int d = ep_direction(c, e);
        e->regs[0x00] = (uint16_t)(v & ~0x40u);
        if (v & 0x40) ep_sync_in(c, e);
        ep_restart(c, e, ct, d);
        return 0;
    }
    case 0x1C: e->etflg &= ~v; return 0;
    case 0x1D:
        if ((v & 1) && !(e->etflg & 1)) { e->etflg |= 1; pie_raise(c, 3, e->idx); }
        if (v & 4) { e->etflg |= 4; if (adc_trigger(c, 5 + 2 * (e->idx - 1)) < 0) return -1; }
        if (v & 8) { e->etflg |= 8; if (adc_trigger(c, 5 + 2 * (e->idx - 1) + 1) < 0) return -1; }
        return 0;
    case 0x17:
        e->tzflg &= ~v;
        if (v & 4) e->tz_forced = 0;
        if (!(e->tzflg & 6)) e->tzflg &= ~1u;
        return 0;
    case 0x18:
        if (v & 6) return ep_trip(c, e, (v & 4) != 0);
        return 0;
    }
    if (off < 0x40) e->regs[off] = (uint16_t)v;
    return 0;
}

static int ep_write(Core *c, NEpwm *e, uint32_t off, uint32_t v) {
    ep_catch_up(c, e);
    if (ep_write_inner(c, e, off, v) < 0) return -1;
    switch (off) { case 0x05: case 0x09: case 0x0A: case 0x19: case 0x1A: case 0x00: case 0x07: ep_rearm(c, e); }
    return c->err ? -1 : 0;
}

static void ep_reset(Core *c, NEpwm *e) {
    int idx = e->idx, pb = e->py_boundary, pt = e->py_tz;
    memset(e, 0, sizeof *e);
    e->idx = idx; e->py_boundary = pb; e->py_tz = pt;
    e->anchor_t = c->cycles;
    e->dir = 1;
    e->ev_t = FP_NONE_T;
    e->regs[0x00] = 0x0083;
}

/* ================================================================ routing */
static int fp_io_read(Core *c, uint32_t a, uint32_t *out) {
    Fp *f = c->fp;
    if (!f || a >= 0x8000 || f->trace) return 0;
    switch (f->route[a >> 6]) {
    case 0: return 0;
    case FP_PIE: if (a < 0x0CE0) return 0; *out = pie_read(c, a - 0x0CE0); return 1;
    case FP_ADC: *out = adc_read(c, a - 0x7100); return 1;
    case FP_CLA: if (a >= 0x1480) return 0; *out = cla_read(c, a - 0x1400); return 1;
    default: {
        int k = f->route[a >> 6] - FP_EPWM + 1;
        *out = ep_read(c, EP(c, k), a - (0x6800 + 0x40 * (uint32_t)(k - 1)));
        return 1;
    }
    }
}

static int fp_io_write(Core *c, uint32_t a, uint32_t v) {
    Fp *f = c->fp;
    if (!f || a >= 0x8000 || f->trace) return 0;
    switch (f->route[a >> 6]) {
    case 0: return 0;
    case FP_PIE: if (a < 0x0CE0) return 0; pie_write(c, a - 0x0CE0, v); return 1;
    case FP_ADC: if (adc_write(c, a - 0x7100, v) < 0) c->err = 1; return 1;
    case FP_CLA:
        if (a >= 0x1480) return 0;
        if (c->eallow) cla_write(c, a - 0x1400, v);
        return 1;
    default: {
        int k = f->route[a >> 6] - FP_EPWM + 1;
        if (ep_write(c, EP(c, k), a - (0x6800 + 0x40 * (uint32_t)(k - 1)), v) < 0) c->err = 1;
        return 1;
    }
    }
}

/* the timed events that are due; 0, or -1 on a Python error */
static int fp_due(Core *c) {
    Fp *f = c->fp;
    for (long guard = 0; guard < (1L << 20) && f->next <= c->cycles; guard++) {
        /* the earliest first; on a tie: ePWMs, ADC EOCs, conversion, CLA end */
        int64_t best = FP_NONE_T; int what = -1;
        for (int k = 1; k <= 7; k++) if (f->ep[k].ev_t < best) { best = f->ep[k].ev_t; what = k; }
        for (int i = 0; i < f->adc.neoc; i++) if (f->adc.eoc[i].t < best) { best = f->adc.eoc[i].t; what = 100 + i; }
        if (f->adc.conv_t < best) { best = f->adc.conv_t; what = 200; }
        if (f->cla.end_t < best) { best = f->cla.end_t; what = 300; }
        if (what < 0 || best > c->cycles) { fp_recalc(c); break; }
        if (what <= 7) {
            if (ep_boundary(c, EP(c, what), f->ep[what].ev_kind) < 0) return -1;
        } else if (what < 200) {
            NEoc e = f->adc.eoc[what - 100];
            for (int j = what - 100 + 1; j < f->adc.neoc; j++) f->adc.eoc[j - 1] = f->adc.eoc[j];
            f->adc.neoc--;
            fp_recalc(c);
            if (adc_eoc(c, &e) < 0) return -1;
        } else if (what == 200) {
            adc_convert(c);
            if (c->err) return -1;
        } else {
            cla_task_end(c);
            if (c->err) return -1;
        }
        fp_recalc(c);
    }
    return c->err ? -1 : 0;
}
