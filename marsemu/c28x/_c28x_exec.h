/* Instruction bodies for _c28x.c, one case per key, mirroring semantics.py. */

static int exec1(Core *c, Entry *e, uint32_t at) {
    uint32_t L = e->L, C = e->C, v, b, r, a2;
    int A = e->A;
    int32_t a;
    uint32_t nxt = (at + e->size) & M22;
    switch (e->key) {
    /* ------------------------------------------------------------ moves */
    case K_MOV_AX_LOC: v = rdl16(c, L); ax_set(c, A, v); nz16(c, v); break;
    case K_MOV_LOC_AX: v = ax_get(c, A); wrl16(c, L, v); if (L == 0xA8 || L == 0xA9) nz16(c, v); break;
    case K_MOV_LOC_AX_COND:
        if (cond(c, e->K)) st16(c, L, ax_get(c, A)); else ea(c, L, 16);
        break;
    case K_MOV_LOC_IMM: st16(c, L, C); break;
    case K_MOV_LOC_0: st16(c, L, 0); break;
    case K_MOV_ACC_LOC: v = ext16(c, rdl16(c, L)); c->acc = v; nz32(c, v); break;
    case K_MOV_ACC_LOC_S: v = ext16(c, rdl16(c, L)) << e->S; c->acc = v; nz32(c, v); break;
    case K_MOV_ACC_LOC_16: v = ext16(c, rdl16(c, L)) << 16; c->acc = v; nz32(c, v); break;
    case K_MOV_ACC_LOC_T: v = ext16(c, rdl16(c, L)) << ((c->xt >> 16) & 15); c->acc = v; nz32(c, v); break;
    case K_MOV_ACC_IMM: v = ext16(c, C) << e->S; c->acc = v; nz32(c, v); break;
    case K_MOVU_ACC_LOC: v = rdl16(c, L); c->acc = v; nz32(c, v); break;
    case K_MOVB_ACC: c->acc = C; nz32(c, C); break;
    case K_MOVB_AX: ax_set(c, A, C); nz16(c, C); break;
    case K_MOVB_XARN: c->xar[e->n] = C; break;
    case K_MOVB_XAR6: c->xar[6] = C; break;
    case K_MOVB_XAR7: c->xar[7] = C; break;
    case K_MOVB_AR6: c->xar[6] = (c->xar[6] & 0xFFFF0000u) | C; break;
    case K_MOVB_AR7: c->xar[7] = (c->xar[7] & 0xFFFF0000u) | C; break;
    case K_MOVL_XAR0_IMM: c->xar[0] = C; break;
    case K_MOVL_XAR1_IMM: c->xar[1] = C; break;
    case K_MOVL_XAR2_IMM: c->xar[2] = C; break;
    case K_MOVL_XAR3_IMM: c->xar[3] = C; break;
    case K_MOVL_XAR4_IMM: c->xar[4] = C; break;
    case K_MOVL_XAR5_IMM: c->xar[5] = C; break;
    case K_MOVL_XAR6_IMM: c->xar[6] = C; break;
    case K_MOVL_XAR7_IMM: c->xar[7] = C; break;
    case K_MOVL_XAR0_LOC: c->xar[0] = rdl32(c, L); break;
    case K_MOVL_XAR1_LOC: c->xar[1] = rdl32(c, L); break;
    case K_MOVL_XAR2_LOC: c->xar[2] = rdl32(c, L); break;
    case K_MOVL_XAR3_LOC: c->xar[3] = rdl32(c, L); break;
    case K_MOVL_XAR4_LOC: c->xar[4] = rdl32(c, L); break;
    case K_MOVL_XAR5_LOC: c->xar[5] = rdl32(c, L); break;
    case K_MOVL_XAR6_LOC: c->xar[6] = rdl32(c, L); break;
    case K_MOVL_XAR7_LOC: c->xar[7] = rdl32(c, L); break;
    case K_MOVL_LOC_XAR0: st32(c, L, c->xar[0]); break;
    case K_MOVL_LOC_XAR1: st32(c, L, c->xar[1]); break;
    case K_MOVL_LOC_XAR2: st32(c, L, c->xar[2]); break;
    case K_MOVL_LOC_XAR3: st32(c, L, c->xar[3]); break;
    case K_MOVL_LOC_XAR4: st32(c, L, c->xar[4]); break;
    case K_MOVL_LOC_XAR5: st32(c, L, c->xar[5]); break;
    case K_MOVL_LOC_XAR6: st32(c, L, c->xar[6]); break;
    case K_MOVL_LOC_XAR7: st32(c, L, c->xar[7]); break;
    case K_MOVL_ACC_LOC: v = rdl32(c, L); c->acc = v; nz32(c, v); break;
    case K_MOVL_LOC_ACC: st32(c, L, c->acc); break;
    case K_MOVL_LOC_ACC_COND: if (cond(c, e->K)) st32(c, L, c->acc); else ea(c, L, 32); break;
    case K_MOVL_P_LOC: c->p = rdl32(c, L); break;
    case K_MOVL_LOC_P: st32(c, L, c->p); break;
    case K_MOVL_P_ACC: c->p = c->acc; break;
    case K_MOVL_XT_LOC: c->xt = rdl32(c, L); break;
    case K_MOVL_LOC_XT: st32(c, L, c->xt); break;
    case K_MOVL_ACC_P: v = PSHIFT(c); c->acc = v; nz32(c, v); break;
    case K_MOVDL: a = ea(c, L, 32); if (a < 0) return run_py(c, at); v = rd32(c, (uint32_t)a); c->xt = v; wr32(c, ((uint32_t)a & ~1u) + 2, v); break;
    case K_MOVW_DP: case K_MOV_DP_IMM: case K_MOVZ_DP: c->dp = C; break;
    case K_MOVZ_ARN: c->xar[e->n] = rdl16(c, L); break;
    case K_MOVZ_AR6: c->xar[6] = rdl16(c, L); break;
    case K_MOVZ_AR7: c->xar[7] = rdl16(c, L); break;
    case K_MOV_AR6_LOC: c->xar[6] = (c->xar[6] & 0xFFFF0000u) | rdl16(c, L); break;
    case K_MOV_AR7_LOC: c->xar[7] = (c->xar[7] & 0xFFFF0000u) | rdl16(c, L); break;
    case K_MOV_LOC_ARN: st16(c, L, c->xar[e->n] & M16); break;
    case K_MOV_XARN_PC: c->xar[e->n] = at; break;
    case K_MOV_T_LOC: c->xt = (c->xt & M16) | (rdl16(c, L) << 16); break;
    case K_MOV_LOC_T: st16(c, L, c->xt >> 16); break;
    case K_MOV_TL_0: c->xt &= 0xFFFF0000u; break;
    case K_MOVX_TL: v = rdl16(c, L); c->xt = s16v(v); break;
    case K_MOV_PH_LOC: c->p = (c->p & M16) | (rdl16(c, L) << 16); break;
    case K_MOV_PL_LOC: c->p = (c->p & 0xFFFF0000u) | rdl16(c, L); break;
    case K_MOV_LOC_P: st16(c, L, PSHIFT(c) & M16); break;
    case K_MOVH_LOC_P: st16(c, L, PSHIFT(c) >> 16); break;
    case K_MOV_LOC_ACC_1: st16(c, L, (c->acc << 1) & M16); break;
    case K_MOV_LOC_ACC_S: st16(c, L, (c->acc << e->S) & M16); break;
    case K_MOVH_LOC_ACC_1: st16(c, L, ((c->acc << 1) >> 16) & M16); break;
    case K_MOVH_LOC_ACC_S: st16(c, L, ((uint32_t)((uint64_t)c->acc << e->S >> 16)) & M16); break;
    case K_MOV_LOC_IER: st16(c, L, c->ier); break;
    case K_MOV_IER_LOC: c->ier = rdl16(c, L); break;
    case K_MOV_LOC_OVC: case K_MOVU_LOC_OVC: st16(c, L, (uint32_t)(c->ovc & 63) << 10); break;
    case K_MOV_PM_AX: c->pm = ax_get(c, A) & 7; break;
    case K_MOV_ABS_LOC: wr16(c, C, rdl16(c, L)); break;
    case K_MOV_LOC_ABS: st16(c, L, rd16(c, C)); break;
    case K_MOVB_LOC_IMM_COND: if (cond(c, e->K)) st16(c, L, C); else ea(c, L, 16); break;
    /* ---------------------------------------------- AX arithmetic and logic */
    case K_ADD_AX_LOC: b = rdl16(c, L); ax_set(c, A, add16(c, ax_get(c, A), b)); break;
    case K_SUB_AX_LOC: b = rdl16(c, L); ax_set(c, A, sub16(c, ax_get(c, A), b)); break;
    case K_AND_AX_LOC: b = rdl16(c, L); r = ax_get(c, A) & b; ax_set(c, A, r); nz16(c, r); break;
    case K_OR_AX_LOC: b = rdl16(c, L); r = ax_get(c, A) | b; ax_set(c, A, r); nz16(c, r); break;
    case K_XOR_AX_LOC: b = rdl16(c, L); r = ax_get(c, A) ^ b; ax_set(c, A, r); nz16(c, r); break;
    case K_AND_AX_LOC_IMM: r = rdl16(c, L) & C; ax_set(c, A, r); nz16(c, r); break;
#define RMW16(expr) \
        a = ea(c, L, 16); v = a < 0 ? reg16_get(c, L) : rd16(c, (uint32_t)a); r = (expr); \
        if (a < 0) reg16_set(c, L, r); else wr16(c, (uint32_t)a, r);
    case K_ADD_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(add16(c, v, ax)); break; }
    case K_SUB_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(sub16(c, v, ax)); break; }
    case K_SUBR_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(sub16(c, ax, v)); break; }
    case K_AND_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(v & ax); nz16(c, r); break; }
    case K_OR_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(v | ax); nz16(c, r); break; }
    case K_XOR_LOC_AX: { uint32_t ax = ax_get(c, A); RMW16(v ^ ax); nz16(c, r); break; }
    case K_ADD_LOC_IMM: RMW16(add16(c, v, C)); break;
    case K_AND_LOC_IMM: RMW16(v & C); nz16(c, r); break;
    case K_OR_LOC_IMM: RMW16(v | C); nz16(c, r); break;
    case K_XOR_LOC_IMM: RMW16(v ^ C); nz16(c, r); break;
    case K_INC: RMW16(add16(c, v, 1)); break;
    case K_DEC: RMW16(sub16(c, v, 1)); break;
    case K_TSET: RMW16(v | (1u << e->B)); c->tc = (v >> e->B) & 1; nz16(c, r); break;
    case K_TCLR: RMW16(v & ~(1u << e->B) & M16); c->tc = (v >> e->B) & 1; nz16(c, r); break;
    case K_ANDB: r = ax_get(c, A) & C; ax_set(c, A, r); nz16(c, r); break;
    case K_ORB: r = ax_get(c, A) | C; ax_set(c, A, r); nz16(c, r); break;
    case K_XORB: r = ax_get(c, A) ^ C; ax_set(c, A, r); nz16(c, r); break;
    case K_ADDB_AX: ax_set(c, A, add16(c, ax_get(c, A), (C & 0x80) ? (C - 256) & M16 : C)); break;
    case K_CMP_AX_LOC: {
        uint32_t x = ax_get(c, A), y = rdl16(c, L);
        c->n = (si16(x) - si16(y)) < 0; c->z = x == y; c->c = x >= y; break; }
    case K_CMPB: { uint32_t x = ax_get(c, A); c->n = (si16(x) - (int32_t)C) < 0; c->z = x == C; c->c = x >= C; break; }
    case K_CMP_LOC_IMM: {
        uint32_t x = rdl16(c, L); c->n = (si16(x) - si16(C)) < 0; c->z = x == C; c->c = x >= C; break; }
    case K_NEG_AX: { uint32_t x = ax_get(c, A); if (x == 0x8000) { c->v = 1; r = 0x8000; } else r = (-x) & M16;
        c->c = x == 0; ax_set(c, A, r); nz16(c, r); break; }
    case K_NOT_AX: r = ax_get(c, A) ^ M16; ax_set(c, A, r); nz16(c, r); break;
    case K_MAX: case K_MIN: {
        int32_t x = si16(ax_get(c, A)), y = si16(rdl16(c, L));
        c->n = (x - y) < 0; c->z = x == y; c->c = (uint32_t)(x & M16) >= (uint32_t)(y & M16);
        if ((e->key == K_MAX && y > x) || (e->key == K_MIN && y < x)) ax_set(c, A, (uint32_t)y & M16);
        break; }
    case K_LSL_AX_IMM: case K_LSR_AX_IMM: case K_ASR_AX_IMM: case K_LSL_AX_T: case K_LSR_AX_T: case K_ASR_AX_T: {
        int k = e->key;
        int nsh = (k == K_LSL_AX_IMM || k == K_LSR_AX_IMM || k == K_ASR_AX_IMM) ? e->S + 1 : (int)((c->xt >> 16) & 15);
        uint32_t x = ax_get(c, A);
        if (nsh == 0) { c->c = 0; r = x; }
        else if (k == K_LSL_AX_IMM || k == K_LSL_AX_T) { r = (x << nsh) & M16; c->c = (x >> (16 - nsh)) & 1; }
        else if (k == K_LSR_AX_IMM || k == K_LSR_AX_T) { r = x >> nsh; c->c = (x >> (nsh - 1)) & 1; }
        else { int32_t sx = si16(x); r = (uint32_t)(sx >> nsh) & M16; c->c = (sx >> (nsh - 1)) & 1; }
        ax_set(c, A, r); nz16(c, r); break; }
    /* ------------------------------------------------ ACC arithmetic */
    case K_ADD_ACC_LOC: acc_add(c, ext16(c, rdl16(c, L)), 0, 0, 1); break;
    case K_ADD_ACC_LOC_S: acc_add(c, ext16(c, rdl16(c, L)) << e->S, 0, 0, 1); break;
    case K_ADD_ACC_LOC_16: acc_add(c, ext16(c, rdl16(c, L)) << 16, 0, 1, 1); break;
    case K_ADD_ACC_LOC_T: acc_add(c, ext16(c, rdl16(c, L)) << ((c->xt >> 16) & 15), 0, 0, 1); break;
    case K_ADD_ACC_IMM: acc_add(c, ext16(c, C) << e->S, 0, 0, 1); break;
    case K_SUB_ACC_LOC: acc_sub(c, ext16(c, rdl16(c, L)), 0, 0, 1); break;
    case K_SUB_ACC_LOC_S: acc_sub(c, ext16(c, rdl16(c, L)) << e->S, 0, 0, 1); break;
    case K_SUB_ACC_LOC_16: acc_sub(c, ext16(c, rdl16(c, L)) << 16, 0, 2, 1); break;
    case K_SUB_ACC_LOC_T: acc_sub(c, ext16(c, rdl16(c, L)) << ((c->xt >> 16) & 15), 0, 0, 1); break;
    case K_SUB_ACC_IMM: acc_sub(c, ext16(c, C) << e->S, 0, 0, 1); break;
    case K_ADDB_ACC: acc_add(c, C, 0, 0, 1); break;
    case K_SUBB_ACC: acc_sub(c, C, 0, 0, 1); break;
    case K_ADDU: acc_add(c, rdl16(c, L), 0, 0, 1); break;
    case K_SUBU: acc_sub(c, rdl16(c, L), 0, 0, 1); break;
    case K_ADDCU: acc_add(c, rdl16(c, L), (uint32_t)c->c, 0, 1); break;
    case K_SBBU: acc_sub(c, rdl16(c, L), 1 - (uint32_t)c->c, 0, 1); break;
    case K_ADDL_ACC_LOC: acc_add(c, rdl32(c, L), 0, 0, 1); break;
    case K_SUBL_ACC_LOC: acc_sub(c, rdl32(c, L), 0, 0, 1); break;
    case K_ADDCL: acc_add(c, rdl32(c, L), (uint32_t)c->c, 0, 1); break;
    case K_SUBBL: acc_sub(c, rdl32(c, L), 1 - (uint32_t)c->c, 0, 1); break;
    case K_ADDL_ACC_P: acc_add(c, PSHIFT(c), 0, 0, 1); break;
    case K_SUBL_ACC_P: acc_sub(c, PSHIFT(c), 0, 0, 1); break;
    case K_ADDUL_ACC: { uint64_t s = (uint64_t)c->acc + rdl32(c, L); c->c = (int)(s >> 32); if (c->c) ovc_count(c, 1);
        c->acc = (uint32_t)s; nz32(c, c->acc); break; }
    case K_SUBUL_ACC: { int64_t s = (int64_t)c->acc - (int64_t)rdl32(c, L); c->c = s >= 0; if (!c->c) ovc_count(c, -1);
        c->acc = (uint32_t)s; nz32(c, c->acc); break; }
    case K_ADDUL_P: { uint64_t s = (uint64_t)c->p + rdl32(c, L); c->c = (int)(s >> 32); if (c->c) ovc_count(c, 1);
        c->p = (uint32_t)s; nz32(c, c->p); break; }
    case K_SUBUL_P: { int64_t s = (int64_t)c->p - (int64_t)rdl32(c, L); c->c = s >= 0; if (!c->c) ovc_count(c, -1);
        c->p = (uint32_t)s; nz32(c, c->p); break; }
    case K_ADDL_LOC_ACC: case K_SUBL_LOC_ACC: case K_SUBRL: {
        a = ea(c, L, 32);
        uint32_t m = a < 0 ? reg32_get(c, L) : rd32(c, (uint32_t)a), x, y, res;
        if (e->key == K_ADDL_LOC_ACC) {
            x = m; y = c->acc; uint64_t s = (uint64_t)x + y; c->c = (int)(s >> 32); res = (uint32_t)s;
            if ((~(x ^ y) & (x ^ res)) & 0x80000000u) { c->v = 1;
                if (c->ovm) res = (x & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu; else ovc_count(c, (x & 0x80000000u) ? -1 : 1); }
        } else {
            if (e->key == K_SUBL_LOC_ACC) { x = m; y = c->acc; } else { x = c->acc; y = m; }
            int64_t s = (int64_t)x - (int64_t)y; c->c = s >= 0; res = (uint32_t)s;
            if (((x ^ y) & (x ^ res)) & 0x80000000u) { c->v = 1;
                if (c->ovm) res = (x & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu; else ovc_count(c, (x & 0x80000000u) ? -1 : 1); }
        }
        nz32(c, res);
        if (a < 0) reg32_set(c, L, res); else wr32(c, (uint32_t)a, res);
        break; }
    case K_CMPL_ACC_LOC: case K_CMPL_ACC_P: {
        uint32_t x = c->acc, y = e->key == K_CMPL_ACC_LOC ? rdl32(c, L) : PSHIFT(c);
        c->n = ((int64_t)si32(x) - (int64_t)si32(y)) < 0; c->z = x == y; c->c = x >= y; break; }
    case K_CMP64: {
        int64_t x = (int64_t)(((uint64_t)c->acc << 32) | c->p);
        c->n = c->v ? !(x < 0) : (x < 0); c->z = x == 0; c->v = 0; break; }
    case K_AND_ACC_LOC: r = c->acc & rdl16(c, L); c->acc = r; nz32(c, r); break;
    case K_OR_ACC_LOC: r = c->acc | rdl16(c, L); c->acc = r; nz32(c, r); break;
    case K_XOR_ACC_LOC: r = c->acc ^ rdl16(c, L); c->acc = r; nz32(c, r); break;
    case K_AND_ACC_IMM: r = c->acc & (C << e->S); c->acc = r; nz32(c, r); break;
    case K_OR_ACC_IMM: r = c->acc | (C << e->S); c->acc = r; nz32(c, r); break;
    case K_XOR_ACC_IMM: r = c->acc ^ (C << e->S); c->acc = r; nz32(c, r); break;
    case K_AND_ACC_IMM16: r = c->acc & (C << 16); c->acc = r; nz32(c, r); break;
    case K_OR_ACC_IMM16: r = c->acc | (C << 16); c->acc = r; nz32(c, r); break;
    case K_XOR_ACC_IMM16: r = c->acc ^ (C << 16); c->acc = r; nz32(c, r); break;
    case K_NEG_ACC: { uint32_t x = c->acc;
        if (x == 0x80000000u) { c->v = 1; r = c->ovm ? 0x7FFFFFFFu : 0x80000000u; } else r = (uint32_t)(-(int64_t)x);
        c->c = x == 0; c->acc = r; nz32(c, r); break; }
    case K_NEGTC: if (c->tc) { uint32_t x = c->acc;
            if (x == 0x80000000u) { c->v = 1; r = c->ovm ? 0x7FFFFFFFu : 0x80000000u; } else r = (uint32_t)(-(int64_t)x);
            c->c = x == 0; c->acc = r; }
        nz32(c, c->acc); break;
    case K_NOT_ACC: c->acc ^= M32; nz32(c, c->acc); break;
    case K_ABS: case K_ABSTC: { uint32_t x = c->acc;
        if (x == 0x80000000u) { c->v = 1; r = c->ovm ? 0x7FFFFFFFu : 0x80000000u; if (e->key == K_ABSTC) c->tc ^= 1; }
        else if (x & 0x80000000u) { r = (uint32_t)(-(int64_t)x); if (e->key == K_ABSTC) c->tc ^= 1; }
        else r = x;
        c->c = 0; c->acc = r; nz32(c, r); break; }
    case K_SAT:
        if (c->ovc > 0) { c->acc = 0x7FFFFFFFu; c->v = 1; } else if (c->ovc < 0) { c->acc = 0x80000000u; c->v = 1; } else c->v = 0;
        c->ovc = 0; c->c = 0; nz32(c, c->acc); break;
    case K_TEST: nz32(c, c->acc); break;
    case K_CSB: { uint32_t x = c->acc, sign = x >> 31; int cnt = 0;
        for (int i = 30; i >= 0; i--) { if (((x >> i) & 1) == sign) cnt++; else break; }
        c->xt = (c->xt & M16) | ((uint32_t)cnt << 16); nz32(c, x); c->tc = (int)sign; break; }
    case K_LSL_ACC_IMM: case K_LSL_ACC_T: case K_LSLL: case K_SFR_IMM: case K_SFR_T: case K_LSRL: case K_ASRL: {
        int k = e->key, nsh;
        if (k == K_LSL_ACC_IMM || k == K_SFR_IMM) nsh = e->S + 1;
        else nsh = (int)((c->xt >> 16) & ((k == K_LSLL || k == K_LSRL || k == K_ASRL) ? 31 : 15));
        uint32_t x = c->acc;
        if (nsh == 0) { c->c = 0; r = x; }
        else if (k == K_LSL_ACC_IMM || k == K_LSL_ACC_T || k == K_LSLL) { r = (uint32_t)((uint64_t)x << nsh); c->c = (x >> (32 - nsh)) & 1; }
        else if (k == K_LSRL || ((k == K_SFR_IMM || k == K_SFR_T) && !c->sxm)) { r = x >> nsh; c->c = (x >> (nsh - 1)) & 1; }
        else { int32_t sx = si32(x); r = (uint32_t)(sx >> nsh); c->c = (sx >> (nsh - 1)) & 1; }
        c->acc = r; nz32(c, r); break; }
    case K_ROL: { uint32_t x = c->acc; r = (x << 1) | (uint32_t)c->c; c->c = x >> 31; c->acc = r; nz32(c, r); break; }
    case K_ROR: { uint32_t x = c->acc; r = (x >> 1) | ((uint32_t)c->c << 31); c->c = x & 1; c->acc = r; nz32(c, r); break; }
    case K_LSL64_IMM: case K_LSL64_T: case K_LSR64_IMM: case K_ASR64_IMM: case K_LSR64_T: case K_ASR64_T: {
        int k = e->key;
        int nsh = (k == K_LSL64_IMM || k == K_LSR64_IMM || k == K_ASR64_IMM) ? e->S + 1 : (int)((c->xt >> 16) & 63);
        uint64_t x = ((uint64_t)c->acc << 32) | c->p, rr;
        if (nsh == 0) { c->c = 0; rr = x; }
        else if (k == K_LSL64_IMM || k == K_LSL64_T) { rr = x << nsh; c->c = (int)((x >> (64 - nsh)) & 1); }
        else if (k == K_LSR64_IMM || k == K_LSR64_T) { rr = x >> nsh; c->c = (int)((x >> (nsh - 1)) & 1); }
        else { int64_t sx = (int64_t)x; rr = (uint64_t)(sx >> nsh); c->c = (int)((sx >> (nsh - 1)) & 1); }
        c->acc = (uint32_t)(rr >> 32); c->p = (uint32_t)rr; c->n = (int)(rr >> 63); c->z = rr == 0; break; }
    case K_NEG64: {
        uint64_t x = ((uint64_t)c->acc << 32) | c->p, rr;
        if (x == (1ull << 63)) { c->v = 1; rr = c->ovm ? (1ull << 63) - 1 : x; } else rr = (uint64_t)(-(int64_t)x);
        c->c = x == 0; c->acc = (uint32_t)(rr >> 32); c->p = (uint32_t)rr; c->n = (int)(rr >> 63); c->z = rr == 0; break; }
    case K_SUBCU: { b = rdl16(c, L); uint64_t a2x = (uint64_t)c->acc << 1; int64_t t = (int64_t)a2x - ((int64_t)b << 16);
        if (t >= 0) { c->acc = (uint32_t)(t + 1); c->c = 1; } else { c->acc = (uint32_t)a2x; c->c = 0; }
        nz32(c, c->acc); break; }
    case K_SUBCUL: { b = rdl32(c, L); int64_t t = ((int64_t)c->acc << 1) + (c->p >> 31) - (int64_t)b;
        if (t >= 0) { c->acc = (uint32_t)t; c->p = (c->p << 1) + 1; c->c = 1; }
        else { uint64_t x = (((uint64_t)c->acc << 32) | c->p) << 1; c->acc = (uint32_t)(x >> 32); c->p = (uint32_t)x; c->c = 0; }
        nz32(c, c->acc); break; }
    case K_MAXL: case K_MINL: { int32_t x = si32(c->acc), y = si32(rdl32(c, L));
        c->n = ((int64_t)x - y) < 0; c->z = x == y; c->c = (uint32_t)x >= (uint32_t)y;
        if ((e->key == K_MAXL && y > x) || (e->key == K_MINL && y < x)) c->acc = (uint32_t)y;
        break; }
    case K_MAXCUL: case K_MINCUL: { b = rdl32(c, L); int32_t hi = si32(c->acc);
        if (e->key == K_MAXCUL) { if (!(hi > 0 || (hi == 0 && c->p >= b)) && (hi < 0 || c->p < b)) c->p = b; }
        else { if (!(hi < 0 || (hi == 0 && c->p <= b))) c->p = b; }
        nz32(c, c->p); break; }
    /* ------------------------------------------------------- multiply */
    case K_MPY_P_T_LOC: c->p = mul_ss(c->xt >> 16, rdl16(c, L)); break;
    case K_MPY_ACC_T_LOC: v = mul_ss(c->xt >> 16, rdl16(c, L)); c->acc = v; nz32(c, v); break;
    case K_MPY_ACC_LOC_IMM: { uint32_t t = rdl16(c, L); c->xt = (c->xt & M16) | (t << 16); v = mul_ss(t, C); c->acc = v; nz32(c, v); break; }
    case K_MPY_P_LOC_IMM: c->p = mul_ss(rdl16(c, L), C); break;
    case K_MPYA_P_T_LOC: case K_MPYS: { b = rdl16(c, L);
        if (e->key == K_MPYS) acc_sub(c, PSHIFT(c), 0, 0, 1); else acc_add(c, PSHIFT(c), 0, 0, 1);
        c->p = mul_ss(c->xt >> 16, b); break; }
    case K_MPYA_P_LOC_IMM: { uint32_t t = rdl16(c, L); acc_add(c, PSHIFT(c), 0, 0, 1);
        c->xt = (c->xt & M16) | (t << 16); c->p = mul_ss(t, C); break; }
    case K_MPYB_P: c->p = (uint32_t)(si16(c->xt >> 16) * (int32_t)C); break;
    case K_MPYB_ACC: v = (uint32_t)(si16(c->xt >> 16) * (int32_t)C); c->acc = v; nz32(c, v); break;
    case K_MPYU_P: c->p = (c->xt >> 16) * rdl16(c, L); break;
    case K_MPYU_ACC: v = (c->xt >> 16) * rdl16(c, L); c->acc = v; nz32(c, v); break;
    case K_MPYXU_P: c->p = (uint32_t)(si16(c->xt >> 16) * (int32_t)rdl16(c, L)); break;
    case K_MPYXU_ACC: v = (uint32_t)(si16(c->xt >> 16) * (int32_t)rdl16(c, L)); c->acc = v; nz32(c, v); break;
    case K_MOVA: case K_MOVS: case K_MOVP: { uint32_t t = rdl16(c, L), ps = PSHIFT(c);
        c->xt = (c->xt & M16) | (t << 16);
        if (e->key == K_MOVA) acc_add(c, ps, 0, 0, 1); else if (e->key == K_MOVS) acc_sub(c, ps, 0, 0, 1); else { c->acc = ps; nz32(c, ps); }
        break; }
    case K_IMPYL_P: c->p = (uint32_t)((int64_t)si32(c->xt) * si32(rdl32(c, L))); break;
    case K_IMPYL_ACC: v = (uint32_t)((int64_t)si32(c->xt) * si32(rdl32(c, L))); c->acc = v; nz32(c, v); break;
    case K_QMPYL_P: c->p = (uint32_t)(((int64_t)si32(c->xt) * si32(rdl32(c, L))) >> 32); break;
    case K_QMPYL_ACC: v = (uint32_t)(((int64_t)si32(c->xt) * si32(rdl32(c, L))) >> 32); c->acc = v; nz32(c, v); break;
    case K_QMPYUL: c->p = (uint32_t)(((uint64_t)c->xt * rdl32(c, L)) >> 32); break;
    case K_QMPYXUL: c->p = (uint32_t)(((__int128)si32(c->xt) * (int64_t)rdl32(c, L)) >> 32); break;
    case K_IMPYXUL: c->p = (uint32_t)((int64_t)si32(c->xt) * (int64_t)rdl32(c, L)); break;
    case K_QMPYAL: { b = rdl32(c, L); acc_add(c, PSHIFT(c), 0, 0, 1); c->p = (uint32_t)(((int64_t)si32(c->xt) * si32(b)) >> 32); break; }
    case K_QMPYSL: { b = rdl32(c, L); acc_sub(c, PSHIFT(c), 0, 0, 1); c->p = (uint32_t)(((int64_t)si32(c->xt) * si32(b)) >> 32); break; }
    /* ----------------------------------------------------- bits, regs */
    case K_TBIT_IMM: c->tc = (rdl16(c, L) >> e->B) & 1; break;
    case K_TBIT_T: c->tc = (rdl16(c, L) >> (15 - ((c->xt >> 16) & 15))) & 1; break;
    case K_ADDB_XAR: c->xar[e->n] += C; break;
    case K_SUBB_XAR: c->xar[e->n] -= C; break;
    case K_ADDB_SP: c->sp = (c->sp + C) & M16; break;
    case K_SUBB_SP: c->sp = (c->sp - C) & M16; break;
    case K_ASP: if (c->sp & 1) { c->sp = (c->sp + 1) & M16; c->spa = 1; } else c->spa = 0; break;
    case K_NASP: if (c->spa) { c->sp = (c->sp - 1) & M16; c->spa = 0; } break;
    case K_PUSH_LOC: push16(c, rdl16(c, L)); break;
    case K_POP_LOC: st16(c, L, pop16(c)); break;
    case K_PUSH_AR1_AR0: push32(c, ((c->xar[1] & M16) << 16) | (c->xar[0] & M16)); break;
    case K_PUSH_AR3_AR2: push32(c, ((c->xar[3] & M16) << 16) | (c->xar[2] & M16)); break;
    case K_PUSH_AR5_AR4: push32(c, ((c->xar[5] & M16) << 16) | (c->xar[4] & M16)); break;
    case K_PUSH_AR1H_AR0H: push32(c, (c->xar[1] & 0xFFFF0000u) | (c->xar[0] >> 16)); break;
    case K_POP_AR1_AR0: v = pop32(c); c->xar[1] = (c->xar[1] & 0xFFFF0000u) | (v >> 16); c->xar[0] = (c->xar[0] & 0xFFFF0000u) | (v & M16); break;
    case K_POP_AR3_AR2: v = pop32(c); c->xar[3] = (c->xar[3] & 0xFFFF0000u) | (v >> 16); c->xar[2] = (c->xar[2] & 0xFFFF0000u) | (v & M16); break;
    case K_POP_AR5_AR4: v = pop32(c); c->xar[5] = (c->xar[5] & 0xFFFF0000u) | (v >> 16); c->xar[4] = (c->xar[4] & 0xFFFF0000u) | (v & M16); break;
    case K_POP_AR1H_AR0H: v = pop32(c); c->xar[1] = (c->xar[1] & M16) | (v & 0xFFFF0000u); c->xar[0] = (c->xar[0] & M16) | ((v & M16) << 16); break;
    case K_PUSH_DP: push16(c, c->dp); break;
    case K_POP_DP: c->dp = pop16(c); break;
    case K_PUSH_ST0: push16(c, st0(c)); break;
    case K_POP_ST0: set_st0(c, pop16(c)); break;
    case K_PUSH_ST1: push16(c, st1(c)); break;
    case K_POP_ST1: set_st1(c, pop16(c)); break;
    case K_PUSH_IFR: push16(c, c->ifr); break;
    case K_POP_IFR: c->ifr = pop16(c); break;
    case K_PUSH_DBGIER: push16(c, c->dbgier); break;
    case K_POP_DBGIER: c->dbgier = pop16(c); break;
    case K_PUSH_DP_ST1: push32(c, (c->dp << 16) | st1(c)); break;
    case K_POP_DP_ST1: v = pop32(c); set_st1(c, v & M16); c->dp = v >> 16; break;
    case K_PUSH_T_ST0: push32(c, (c->xt & 0xFFFF0000u) | st0(c)); break;
    case K_POP_T_ST0: v = pop32(c); set_st0(c, v & M16); c->xt = (c->xt & M16) | (v & 0xFFFF0000u); break;
    case K_PUSH_P: push32(c, c->p); break;
    case K_POP_P: c->p = pop32(c); break;
    case K_PUSH_RPC: push16(c, c->rpc & M16); push16(c, c->rpc >> 16); break;
    case K_POP_RPC: { uint32_t h = pop16(c); c->rpc = ((h << 16) | pop16(c)) & M22; break; }
    case K_PREAD: st16(c, L, rd16(c, (c->xar[7] + (uint32_t)c->rpt_i) & M22)); break;
    case K_PWRITE: wr16(c, (c->xar[7] + (uint32_t)c->rpt_i) & M22, rdl16(c, L)); break;
    case K_XPREAD_PMA: st16(c, L, rd16(c, (0x3F0000u | C) + (uint32_t)c->rpt_i)); break;
    /* ------------------------------------------------------- status */
    case K_SETC: case K_CLRC: {
        int val = e->key == K_SETC;
        if (C & 1) c->sxm = val; if (C & 2) c->ovm = val; if (C & 4) c->tc = val; if (C & 8) c->c = val;
        if (C & 16) c->intm = val; if (C & 32) c->dbgm = val; if (C & 128) c->vmap = val;
        if (C & 64) { if (val) return run_py(c, at); c->page0 = 0; }
        break; }
    case K_CLRC_AMODE: c->amode = 0; break;
    case K_SETC_OBJMODE: c->objmode = 1; break;
    case K_CLRC_OBJMODE: c->objmode = 0; break;
    case K_SETC_M0M1MAP: c->m0m1map = 1; break;
    case K_CLRC_M0M1MAP: c->m0m1map = 0; break;
    case K_SETC_XF: c->xf = 1; break;
    case K_CLRC_XF: c->xf = 0; break;
    case K_CLRC_OVC: c->ovc = 0; break;
    case K_SPM: c->pm = e->S; break;
    case K_EALLOW: c->eallow = 1; break;
    case K_EDIS: c->eallow = 0; break;
    case K_AND_IER: c->ier &= C; break;
    case K_OR_IER: c->ier |= C; break;
    case K_AND_IFR: c->ifr &= C; break;
    case K_OR_IFR: c->ifr |= C; break;
    case K_NOP: if (L) ea(c, L, 16); break;
    case K_ESTOP0: case K_ESTOP1: case K_ABORTI: break;
    case K_ZAPA: c->acc = c->p = 0; c->ovc = 0; c->n = 0; c->z = 1; break;
    case K_IACK:
        if (c->cb_iack != Py_None) {
            sync_clock(c);
            PyObject *rr = PyObject_CallFunction(c->cb_iack, "k", (unsigned long)C);
            if (!rr) return -1;
            Py_DECREF(rr);
        }
        break;
    /* ------------------------------------------------- control flow */
    case K_B: case K_BF: if (e->K == 15 || cond(c, e->K)) c->pc = (at + (uint32_t)si16(C)) & M22; break;
    case K_SB: if (e->K == 15 || cond(c, e->K)) c->pc = (at + (uint32_t)(int32_t)(int8_t)C) & M22; break;
    case K_SBF_EQ: if (c->z) c->pc = (at + (uint32_t)(int32_t)(int8_t)C) & M22; break;
    case K_SBF_NEQ: if (!c->z) c->pc = (at + (uint32_t)(int32_t)(int8_t)C) & M22; break;
    case K_SBF_TC: if (c->tc) c->pc = (at + (uint32_t)(int32_t)(int8_t)C) & M22; break;
    case K_SBF_NTC: if (!c->tc) c->pc = (at + (uint32_t)(int32_t)(int8_t)C) & M22; break;
    case K_BANZ: { uint32_t x = c->xar[e->n] & M16; c->xar[e->n] = (c->xar[e->n] & 0xFFFF0000u) | ((x - 1) & M16);
        if (x) c->pc = (at + (uint32_t)si16(C)) & M22; break; }
    case K_BAR: { int eq = (c->xar[e->n] & M16) == (c->xar[e->m] & M16);
        if (eq != (e->B != 0)) c->pc = (at + (uint32_t)si16(C)) & M22; break; }
    case K_LB: c->pc = C & M22; break;
    case K_LB_XAR7: c->pc = c->xar[7] & M22; break;
    case K_LCR: push16(c, c->rpc & M16); push16(c, c->rpc >> 16); c->rpc = nxt; c->pc = C & M22; break;
    case K_LCR_XARN: push16(c, c->rpc & M16); push16(c, c->rpc >> 16); c->rpc = nxt; c->pc = c->xar[e->n] & M22; break;
    case K_LRETR: { c->pc = c->rpc; uint32_t h = pop16(c); c->rpc = ((h << 16) | pop16(c)) & M22; break; }
    case K_LC: push16(c, nxt & M16); push16(c, nxt >> 16); c->pc = C & M22; break;
    case K_LC_XAR7: push16(c, nxt & M16); push16(c, nxt >> 16); c->pc = c->xar[7] & M22; break;
    case K_LRET: case K_LRETE: { uint32_t h = pop16(c); c->pc = ((h << 16) | pop16(c)) & M22; if (e->key == K_LRETE) c->intm = 0; break; }
    case K_FFC: c->xar[7] = nxt; c->pc = C & M22; break;
    case K_IRET: iret(c); break;
    case K_IDLE: c->idlestat = 1; c->idle_wait = 1; break;
    case K_RPT_IMM: c->rptc = C; c->rpt_pending = 1; break;
    case K_RPT_LOC: c->rptc = rdl16(c, L); c->rpt_pending = 1; break;
    case K_NORM_XARN: case K_NORM_ACC_IND: { uint32_t x = c->acc;
        if (x == 0) c->tc = 0;
        else if ((x >> 31) == ((x >> 30) & 1)) { c->acc = x << 1; c->tc = 0;
            if (e->key == K_NORM_XARN) c->xar[e->n] += e->B ? 1 : (uint32_t)-1; }
        else c->tc = 1;
        nz32(c, c->acc); break; }
    default:
        return run_py(c, at);
    }
    return c->err ? -1 : 0;
}
