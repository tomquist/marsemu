/* Python interface to the native F2803x peripherals (_f2803x.h); used by f2803x_native.py. */

enum {
    OP_POLL = 1, OP_NEXT, OP_PIE_RAISE, OP_PIE_READ, OP_PIE_WRITE, OP_EP_READ, OP_EP_WRITE, OP_EP_DUTY,
    OP_EP_TRIP, OP_EP_RESET, OP_EP_COUNTER, OP_EP_DIRECTION, OP_EP_PERIOD, OP_EP_FLAGS, OP_ADC_READ,
    OP_ADC_WRITE, OP_ADC_RESET, OP_ADC_TRIGGER, OP_ADC_TEMP, OP_CLA_READ, OP_CLA_WRITE, OP_CLA_FORCE,
    OP_CLA_IACK, OP_CLA_IRQ, OP_CLA_RESET, OP_TRACE, OP_PIE_RESET, OP_PIE_CLEAR
};

static int fp_sync_in(Core *c) {
    PyObject *cv = PyObject_GetAttr(c->clock, s_cycles);
    if (!cv) return -1;
    c->cycles = PyLong_AsLongLong(cv);
    Py_DECREF(cv);
    return 0;
}

/* fp_enable(core, [(page, code), ...]): native peripherals on these IO pages */
static PyObject *fp_enable(PyObject *self, PyObject *args) {
    PyObject *core, *routes;
    if (!PyArg_ParseTuple(args, "O!O", &CoreType, &core, &routes)) return NULL;
    Core *c = (Core *)core;
    if (!c->fp) {
        c->fp = PyMem_Calloc(1, sizeof(Fp));
        if (!c->fp) return PyErr_NoMemory();
        for (int k = 1; k <= 7; k++) { c->fp->ep[k].idx = k; c->fp->ep[k].ev_t = FP_NONE_T; }
        c->fp->adc.conv_t = FP_NONE_T;
        c->fp->adc.rr = 15;
        c->fp->cla.end_t = FP_NONE_T;
        c->fp->cla.last_unknown = -1;
        c->fp->next = FP_NONE_T;
    }
    Py_ssize_t n = PySequence_Size(routes);
    for (Py_ssize_t i = 0; i < n; i++) {
        PyObject *it = PySequence_GetItem(routes, i);
        unsigned long pg, code;
        if (!it || !PyArg_ParseTuple(it, "kk", &pg, &code)) { Py_XDECREF(it); return NULL; }
        Py_DECREF(it);
        if (pg < 0x200) c->fp->route[pg] = (uint8_t)code;
    }
    Py_RETURN_NONE;
}

static void set_cb(PyObject **slot, PyObject *v) {
    Py_XDECREF(*slot);
    if (v == Py_None) { *slot = NULL; return; }
    Py_INCREF(v);
    *slot = v;
}

/* fp_callbacks(core, sample, listeners, boundary, tz): Python callables or None */
static PyObject *fp_callbacks(PyObject *self, PyObject *args) {
    PyObject *core, *sample, *listeners, *boundary, *tz;
    if (!PyArg_ParseTuple(args, "O!OOOO", &CoreType, &core, &sample, &listeners, &boundary, &tz)) return NULL;
    Fp *f = ((Core *)core)->fp;
    if (!f) { PyErr_SetString(PyExc_RuntimeError, "native peripherals not enabled"); return NULL; }
    set_cb(&f->adc.sample, sample);
    set_cb(&f->adc.listeners, listeners);
    set_cb(&f->boundary_cb, boundary);
    set_cb(&f->tz_cb, tz);
    Py_RETURN_NONE;
}

/* fp(core, op, a=0, b=0, c=0) -> int, float or None */
static PyObject *fp_op(PyObject *self, PyObject *args) {
    PyObject *core;
    int op;
    long long a = 0, b = 0, d = 0;
    if (!PyArg_ParseTuple(args, "O!i|LLL", &CoreType, &core, &op, &a, &b, &d)) return NULL;
    Core *c = (Core *)core;
    Fp *f = c->fp;
    if (!f) { PyErr_SetString(PyExc_RuntimeError, "native peripherals not enabled"); return NULL; }
    if (fp_sync_in(c) < 0) return NULL;
    c->err = 0;
    long long r = 0;
    int k = (int)a;
    if (op >= OP_EP_READ && op <= OP_EP_FLAGS && (k < 1 || k > 7)) {
        PyErr_SetString(PyExc_ValueError, "ePWM index");
        return NULL;
    }
    switch (op) {
    case OP_POLL: if (fp_due(c) < 0) goto fail; break;
    case OP_NEXT: r = f->next; break;
    case OP_PIE_RAISE: pie_raise(c, (int)a, (int)b); break;
    case OP_PIE_CLEAR: f->pie.ifr[a & 15] &= ~(1u << (b - 1)); break;
    case OP_PIE_READ: r = pie_read(c, (uint32_t)a); break;
    case OP_PIE_WRITE: pie_write(c, (uint32_t)a, (uint32_t)b & M16); break;
    case OP_PIE_RESET: {
        uint64_t counts[13][9];
        memcpy(counts, f->pie.counts, sizeof counts);
        memset(&f->pie, 0, sizeof f->pie);
        memcpy(f->pie.counts, counts, sizeof counts);
        break;
    }
    case OP_EP_READ: r = ep_read(c, EP(c, k), (uint32_t)b); break;
    case OP_EP_WRITE: if (ep_write(c, EP(c, k), (uint32_t)b, (uint32_t)d & M16) < 0) goto fail; break;
    case OP_EP_DUTY: {
        double v = ep_duty(c, EP(c, k), (int)b);
        if (c->err) goto fail;
        return PyFloat_FromDouble(v);
    }
    case OP_EP_TRIP: if (ep_trip(c, EP(c, k), (int)b) < 0) goto fail; break;
    case OP_EP_RESET: ep_reset(c, EP(c, k)); fp_recalc(c); break;
    case OP_EP_COUNTER: r = ep_counter(c, EP(c, k)); break;
    case OP_EP_DIRECTION: r = ep_direction(c, EP(c, k)); break;
    case OP_EP_PERIOD: r = ep_period_cycles(EP(c, k)); break;
    case OP_EP_FLAGS:
        EP(c, k)->py_boundary = (int)b;
        EP(c, k)->py_tz = (int)d;
        ep_rearm(c, EP(c, k));
        break;
    case OP_ADC_READ: r = adc_read(c, (uint32_t)a); break;
    case OP_ADC_WRITE: if (adc_write(c, (uint32_t)a, (uint32_t)b & M16) < 0) goto fail; break;
    case OP_ADC_RESET: adc_reset(c); fp_recalc(c); break;
    case OP_ADC_TRIGGER: if (adc_trigger(c, (int)a) < 0) goto fail; break;
    case OP_ADC_TEMP: f->adc.temp_raw = (uint32_t)a; break;
    case OP_CLA_READ: r = cla_read(c, (uint32_t)a); break;
    case OP_CLA_WRITE: cla_write(c, (uint32_t)a, (uint32_t)b & M16); break;
    case OP_CLA_FORCE: cla_force(c, (uint32_t)a); break;
    case OP_CLA_IACK: cla_iack(c, (uint32_t)a); break;
    case OP_CLA_IRQ: cla_peripheral_irq(c, (int)a, (int)b); break;
    case OP_CLA_RESET: {
        NCla *x = &f->cla;
        uint64_t tc[9], ty[9], faults = x->faults;
        memcpy(tc, x->task_counts, sizeof tc);
        memcpy(ty, x->task_cycles, sizeof ty);
        memset(x, 0, sizeof *x);
        memcpy(x->task_counts, tc, sizeof tc);
        memcpy(x->task_cycles, ty, sizeof ty);
        x->faults = faults;
        x->end_t = FP_NONE_T;
        x->last_unknown = -1;
        memset(&c->cla, 0, sizeof c->cla);
        fp_recalc(c);
        break;
    }
    case OP_TRACE: f->trace = (int)a; break;
    default: PyErr_SetString(PyExc_ValueError, "unknown op"); return NULL;
    }
    if (c->err) goto fail;
    if (op == OP_NEXT) return PyLong_FromLongLong(r);
    return PyLong_FromLongLong(r);
fail:
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_RuntimeError, "native peripheral error");
    return NULL;
}

/* io_cacheable(core, addr, on): reads of this IO word are served from the cache
   until the core next calls into Python (the model's value has no side effect
   and does not depend on time) */
static PyObject *io_cacheable(PyObject *self, PyObject *args) {
    PyObject *core;
    unsigned long a;
    int on;
    if (!PyArg_ParseTuple(args, "O!kp", &CoreType, &core, &a, &on)) return NULL;
    Core *c = (Core *)core;
    if (a >= IOC_SIZE) { PyErr_SetString(PyExc_ValueError, "address"); return NULL; }
    if (!c->ioc) {
        c->ioc = PyMem_Calloc(IOC_SIZE, sizeof(IoCache));
        if (!c->ioc) return PyErr_NoMemory();
    }
    c->ioc[a].on = (uint8_t)on;
    c->ioc[a].epoch = c->epoch - 1;
    Py_RETURN_NONE;
}

static PyObject *u64_list(const uint64_t *v, int n) {
    PyObject *l = PyList_New(n);
    if (!l) return NULL;
    for (int i = 0; i < n; i++) PyList_SET_ITEM(l, i, PyLong_FromUnsignedLongLong(v[i]));
    return l;
}

static PyObject *u32_list(const uint32_t *v, int n) {
    PyObject *l = PyList_New(n);
    if (!l) return NULL;
    for (int i = 0; i < n; i++) PyList_SET_ITEM(l, i, PyLong_FromUnsignedLong(v[i]));
    return l;
}

static PyObject *u16_list(const uint16_t *v, int n) {
    PyObject *l = PyList_New(n);
    if (!l) return NULL;
    for (int i = 0; i < n; i++) PyList_SET_ITEM(l, i, PyLong_FromLong(v[i]));
    return l;
}

/* fp_state(core, what, k=0) -> dict: "pie", "ep" (ePWMk), "adc", "cla", "counts" */
static PyObject *fp_state(PyObject *self, PyObject *args) {
    PyObject *core;
    const char *what;
    int k = 0;
    if (!PyArg_ParseTuple(args, "O!s|i", &CoreType, &core, &what, &k)) return NULL;
    Core *c = (Core *)core;
    Fp *f = c->fp;
    if (!f) { PyErr_SetString(PyExc_RuntimeError, "native peripherals not enabled"); return NULL; }
    if (!strcmp(what, "pie")) {
        PyObject *counts = PyDict_New();
        for (int g = 1; g <= 12; g++)
            for (int y = 1; y <= 8; y++)
                if (f->pie.counts[g][y]) {
                    PyObject *key = Py_BuildValue("(ii)", g, y), *v = PyLong_FromUnsignedLongLong(f->pie.counts[g][y]);
                    PyDict_SetItem(counts, key, v);
                    Py_DECREF(key); Py_DECREF(v);
                }
        PyObject *ier = u32_list(f->pie.ier, 13), *ifr = u32_list(f->pie.ifr, 13);
        PyObject *d = Py_BuildValue("{sNsNsksksN}", "ier", ier, "ifr", ifr, "ack", (unsigned long)f->pie.ack,
                                    "ctrl", (unsigned long)f->pie.ctrl, "counts", counts);
        return d;
    }
    if (!strcmp(what, "ep")) {
        if (k < 1 || k > 7) { PyErr_SetString(PyExc_ValueError, "ePWM index"); return NULL; }
        NEpwm *e = EP(c, k);
        return Py_BuildValue("{sLsLsLsLsLsLsisisisLsLsisisisisksksisLsLsLsLsN}",
                             "prd", (long long)e->prd, "prd_shadow", (long long)e->prd_shadow,
                             "cmpa", (long long)e->cmpa, "cmpa_sh", (long long)e->cmpa_sh,
                             "cmpb", (long long)e->cmpb, "cmpb_sh", (long long)e->cmpb_sh,
                             "prd_pend", e->prd_pend, "cmpa_pend", e->cmpa_pend, "cmpb_pend", e->cmpb_pend,
                             "anchor_t", (long long)e->anchor_t, "anchor_ctr", (long long)e->anchor_ctr, "dir", e->dir,
                             "et_int_cnt", e->et_int_cnt, "et_soca_cnt", e->et_soca_cnt, "et_socb_cnt", e->et_socb_cnt,
                             "etflg", (unsigned long)e->etflg, "tzflg", (unsigned long)e->tzflg,
                             "tz_forced", e->tz_forced, "periods", (long long)e->periods, "ints", (long long)e->ints,
                             "socas", (long long)e->socas, "ev_t", (long long)(e->ev_t == FP_NONE_T ? -1 : e->ev_t),
                             "regs", u16_list(e->regs, 0x40));
    }
    if (!strcmp(what, "adc")) {
        NAdc *d = &f->adc;
        return Py_BuildValue("{sNsksksksisKsLsksN}", "result", u16_list(d->result, 16),
                             "pending", (unsigned long)d->pending, "intflg", (unsigned long)d->intflg,
                             "intovf", (unsigned long)d->intovf, "rr", d->rr,
                             "conversions", (unsigned long long)d->conversions, "busy_until", (long long)d->busy_until,
                             "temp_raw", (unsigned long)d->temp_raw, "socctl", u16_list(d->socctl, 16));
    }
    if (!strcmp(what, "cla")) {
        NCla *a = &f->cla;
        return Py_BuildValue("{sksksksksisNsNsKsLsN}", "mifr", (unsigned long)a->mifr, "miovf", (unsigned long)a->miovf,
                             "mier", (unsigned long)a->mier, "mirun", (unsigned long)a->mirun, "running", a->running,
                             "task_counts", u64_list(a->task_counts, 9), "task_cycles", u64_list(a->task_cycles, 9),
                             "faults", (unsigned long long)a->faults, "last_unknown", (long long)a->last_unknown,
                             "regs", u16_list(a->regs, 0x80));
    }
    if (!strcmp(what, "counts")) {
        return Py_BuildValue("{sKsN}", "exc_count", (unsigned long long)f->exc_count, "int_count", u64_list(f->int_count, 32));
    }
    PyErr_SetString(PyExc_ValueError, "unknown state");
    return NULL;
}
