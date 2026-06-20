/*
 * Host unit tests for the AES70 OCP.1 codec, command router and access control.
 *
 * These run on the build host (plain gcc, no ESP-IDF): the transport and mDNS
 * modules are stubbed, and the tests drive aes70_route_command_pdu() directly
 * with hand-built OCP.1 command PDUs, asserting the response status and values.
 * The focus is the security-critical logic (PermissionDenied / Locked) and the
 * wire codec.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "aes70.h"
#include "aes70_object.h"
#include "aes70_ocp1.h"
#include "aes70_internal.h"

static int g_checks, g_fails;
#define CHECK(cond, ...) do { \
    g_checks++; \
    if (!(cond)) { g_fails++; printf("  FAIL %s:%d: ", __func__, __LINE__); \
                   printf(__VA_ARGS__); printf("\n"); } \
} while (0)

/* OcaMethod levels/indices used by the tests. */
enum { L_ROOT = 1, L_ACT = 4 };
enum { ROOT_LOCK = 3, ROOT_UNLOCK = 4 };
enum { GET_PRIMARY = 1, SET_PRIMARY = 2 };   /* Gain/Filter: Get* = 1, Set* = 2 */

/* Build one Command, route it as CmdRrq, and return the response status. If
 * `rparams`/`rplen` are non-NULL, copy the response parameter bytes out. */
static uint8_t route_cmd(aes70_device_t *dev, int ci, uint32_t ono,
                         uint16_t level, uint16_t index,
                         const uint8_t *params, size_t plen,
                         uint8_t *rparams, size_t *rplen)
{
    uint8_t data[128];
    ocp1_wr_t w; ocp1_wr_init(&w, data, sizeof data);
    ocp1_wr_u32(&w, (uint32_t)(17 + plen));     /* inclusive command size */
    ocp1_wr_u32(&w, 0x11223344);                /* handle */
    ocp1_wr_u32(&w, ono);                        /* target ONo */
    ocp1_wr_u16(&w, level);
    ocp1_wr_u16(&w, index);
    ocp1_wr_u8(&w, plen ? 1 : 0);                /* parameterCount */
    for (size_t i = 0; i < plen; i++) ocp1_wr_u8(&w, params[i]);

    uint8_t out[256];
    size_t rlen = aes70_route_command_pdu(dev, ci, OCP1_CMD_RRQ, 1,
                                          data, w.off, out, sizeof out);
    if (rlen < 20) return 0xEE;                  /* malformed/no response */
    if (rparams && rplen) {
        uint32_t msz = ocp1_rd32(out + OCP1_HEADER_LEN);   /* inclusive msg size */
        size_t pl = (msz >= 10) ? msz - 10 : 0;            /* minus size+handle+status+pc */
        memcpy(rparams, out + 20, pl);
        *rplen = pl;
    }
    return out[18];                              /* status byte */
}

static void conn_set(aes70_device_t *dev, int ci, bool privileged)
{
    dev->conns[ci].in_use     = true;
    dev->conns[ci].privileged = privileged;
}

static void f32_to_params(float v, uint8_t *p)
{
    uint8_t b[8]; ocp1_wr_t w; ocp1_wr_init(&w, b, sizeof b);
    ocp1_wr_f32(&w, v); memcpy(p, b, 4);
}
static float params_to_f32(const uint8_t *p)
{
    ocp1_rd_t r; ocp1_rd_init(&r, p, 4); return ocp1_rd_f32(&r);
}
static uint16_t resp_u16(const uint8_t *p)
{
    ocp1_rd_t r; ocp1_rd_init(&r, p, 2); return ocp1_rd_u16(&r);
}

/* ---- Tests -------------------------------------------------------------- */

static void test_codec(void)
{
    uint8_t buf[64];
    ocp1_wr_t w; ocp1_wr_init(&w, buf, sizeof buf);
    ocp1_wr_u8(&w, 0x3B);
    ocp1_wr_u16(&w, 0x0004);
    ocp1_wr_u32(&w, 0xDEADBEEF);
    ocp1_wr_f32(&w, -6.5f);
    ocp1_wr_string(&w, "hello");
    CHECK(!w.err, "writer overflowed");

    ocp1_rd_t r; ocp1_rd_init(&r, buf, w.off);
    CHECK(ocp1_rd_u8(&r) == 0x3B, "u8 sync");
    CHECK(ocp1_rd_u16(&r) == 0x0004, "u16 ver");
    CHECK(ocp1_rd_u32(&r) == 0xDEADBEEF, "u32");
    CHECK(fabsf(ocp1_rd_f32(&r) + 6.5f) < 1e-6f, "f32");
    /* OcaString = u16 codepoint count + UTF-8; "hello" is 5 ASCII bytes. */
    uint16_t slen = ocp1_rd_u16(&r);
    CHECK(slen == 5, "string length prefix = %u (want 5)", slen);
}

static void test_method_is_write(void)
{
    /* Pure predicate: setters vs getters across representative classes. */
    aes70_device_config_t cfg; aes70_device_config_default(&cfg);
    cfg.device_name = "t"; cfg.manufacturer = "t"; cfg.model = "t"; cfg.enable_mdns = false;
    aes70_device_handle_t dev; aes70_device_start(&cfg, &dev);

    struct aes70_object *gain = (struct aes70_object *)
        aes70_gain_create(dev, NULL, "g", -60, 12, 0);
    struct aes70_object *flt = (struct aes70_object *)
        aes70_filter_create(dev, NULL, "f", 1, 1, 1000.f, 4);
    struct aes70_object *dyn = (struct aes70_object *)
        aes70_dynamics_create(dev, NULL, "d", 1);

    CHECK(aes70_method_is_write(gain, L_ACT, SET_PRIMARY),  "SetGain is a write");
    CHECK(!aes70_method_is_write(gain, L_ACT, GET_PRIMARY), "GetGain is not a write");
    CHECK(aes70_method_is_write(gain, 2, 2),  "SetEnabled is a write");
    CHECK(aes70_method_is_write(gain, 2, 9),  "SetLabel is a write");
    CHECK(!aes70_method_is_write(gain, 2, 1), "GetEnabled is not a write");
    CHECK(aes70_method_is_write(gain, L_ROOT, ROOT_LOCK), "Lock is a write");
    CHECK(aes70_method_is_write(flt, L_ACT, 4),  "SetPassband is a write");
    CHECK(!aes70_method_is_write(flt, L_ACT, 3), "GetPassband is not a write");
    CHECK(aes70_method_is_write(dyn, L_ACT, 6),  "SetRatio is a write");
    CHECK(!aes70_method_is_write(dyn, L_ACT, 5), "GetRatioRange is not a write");

    aes70_device_stop(dev);
}

static void test_router_and_security(void)
{
    aes70_device_config_t cfg; aes70_device_config_default(&cfg);
    cfg.device_name = "DSP"; cfg.manufacturer = "M"; cfg.model = "X"; cfg.enable_mdns = false;
    aes70_device_handle_t dev; aes70_device_start(&cfg, &dev);

    aes70_object_handle_t gain = aes70_gain_create(dev, NULL, "Volume", -60, 12, 0);
    aes70_object_handle_t flt  = aes70_filter_create(dev, NULL, "Crossover", 1, 1, 1000.f, 4);
    aes70_object_set_secured(flt, true);                 /* protect the crossover */

    uint32_t gain_ono = aes70_object_ono(gain);
    uint32_t flt_ono  = aes70_object_ono(flt);
    CHECK(aes70_object_is_secured(flt), "filter reports secured");
    CHECK(!aes70_object_is_secured(gain), "gain reports not secured");

    conn_set((aes70_device_t *)dev, 0, false);           /* unprivileged */
    conn_set((aes70_device_t *)dev, 1, true);            /* privileged */

    uint8_t p[8]; size_t rl; uint8_t rp[16]; uint8_t st;

    /* GetGain returns Gain, MinGain, MaxGain (3 x f32); the first is the value. */
    st = route_cmd((aes70_device_t *)dev, 0, gain_ono, L_ACT, GET_PRIMARY, NULL, 0, rp, &rl);
    CHECK(st == AES70_OK, "GetGain status = %u", st);
    CHECK(rl == 12, "GetGain param length = %zu (want 12)", rl);
    CHECK(fabsf(params_to_f32(rp) - 0.f) < 1e-6f, "GetGain value = %.3f", params_to_f32(rp));

    /* SetGain on an unsecured object from an unprivileged session: allowed. */
    f32_to_params(-6.f, p);
    st = route_cmd((aes70_device_t *)dev, 0, gain_ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "SetGain status = %u", st);
    CHECK(fabsf(aes70_gain_get(gain) + 6.f) < 1e-6f, "gain now -6 (got %.2f)", aes70_gain_get(gain));

    /* SetFrequency on the SECURED filter from an unprivileged session: denied. */
    f32_to_params(2000.f, p);
    st = route_cmd((aes70_device_t *)dev, 0, flt_ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_PERMISSION_DENIED, "secured SetFrequency unprivileged = %u (want 15)", st);
    CHECK(fabsf(aes70_filter_get_frequency(flt) - 1000.f) < 1e-3f, "frequency unchanged after deny");

    /* Reads of a secured object are still allowed unprivileged. */
    st = route_cmd((aes70_device_t *)dev, 0, flt_ono, L_ACT, GET_PRIMARY, NULL, 0, rp, &rl);
    CHECK(st == AES70_OK, "GetFrequency on secured object unprivileged = %u", st);

    /* The same write from a privileged session succeeds. */
    f32_to_params(2000.f, p);
    st = route_cmd((aes70_device_t *)dev, 1, flt_ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "secured SetFrequency privileged = %u", st);
    CHECK(fabsf(aes70_filter_get_frequency(flt) - 2000.f) < 1e-3f, "frequency now 2000");

    aes70_device_stop(dev);
}

static void test_locking(void)
{
    aes70_device_config_t cfg; aes70_device_config_default(&cfg);
    cfg.device_name = "DSP"; cfg.manufacturer = "M"; cfg.model = "X"; cfg.enable_mdns = false;
    aes70_device_handle_t dev; aes70_device_start(&cfg, &dev);

    aes70_object_handle_t gain = aes70_gain_create(dev, NULL, "Volume", -60, 12, 0);
    uint32_t ono = aes70_object_ono(gain);

    conn_set((aes70_device_t *)dev, 0, false);
    conn_set((aes70_device_t *)dev, 1, false);

    uint8_t p[8]; uint8_t st;

    /* conn 0 takes a no-read-write lock. */
    st = route_cmd((aes70_device_t *)dev, 0, ono, L_ROOT, ROOT_LOCK, NULL, 0, NULL, NULL);
    CHECK(st == AES70_OK, "Lock status = %u", st);

    /* conn 1 (not the owner) is rejected with Locked. */
    f32_to_params(3.f, p);
    st = route_cmd((aes70_device_t *)dev, 1, ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_LOCKED, "non-owner write = %u (want 3)", st);

    /* the owner (conn 0) can still write. */
    st = route_cmd((aes70_device_t *)dev, 0, ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "owner write = %u", st);

    /* after unlock, anyone can write again. */
    st = route_cmd((aes70_device_t *)dev, 0, ono, L_ROOT, ROOT_UNLOCK, NULL, 0, NULL, NULL);
    CHECK(st == AES70_OK, "Unlock status = %u", st);
    st = route_cmd((aes70_device_t *)dev, 1, ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "write after unlock = %u", st);

    /* dropping the lock-owner connection releases its locks. */
    route_cmd((aes70_device_t *)dev, 0, ono, L_ROOT, ROOT_LOCK, NULL, 0, NULL, NULL);
    aes70_locks_drop_conn((aes70_device_t *)dev, 0);
    st = route_cmd((aes70_device_t *)dev, 1, ono, L_ACT, SET_PRIMARY, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "write after owner dropped = %u", st);

    aes70_device_stop(dev);
}

static void test_bad_ono(void)
{
    aes70_device_config_t cfg; aes70_device_config_default(&cfg);
    cfg.device_name = "DSP"; cfg.manufacturer = "M"; cfg.model = "X"; cfg.enable_mdns = false;
    aes70_device_handle_t dev; aes70_device_start(&cfg, &dev);
    conn_set((aes70_device_t *)dev, 0, false);

    uint8_t st = route_cmd((aes70_device_t *)dev, 0, 0x09999999, L_ACT, GET_PRIMARY, NULL, 0, NULL, NULL);
    CHECK(st == AES70_BAD_ONO, "unknown ONo = %u (want 5)", st);

    aes70_device_stop(dev);
}

static void test_grouper(void)
{
    aes70_device_config_t cfg; aes70_device_config_default(&cfg);
    cfg.device_name = "DSP"; cfg.manufacturer = "M"; cfg.model = "X"; cfg.enable_mdns = false;
    aes70_device_handle_t dev; aes70_device_start(&cfg, &dev);

    aes70_object_handle_t g1 = aes70_gain_create(dev, NULL, "Ch1", -80, 12, 0);
    aes70_object_handle_t g2 = aes70_gain_create(dev, NULL, "Ch2", -80, 12, 0);
    aes70_object_handle_t grp = aes70_grouper_create(dev, NULL, "GainGroup", AES70_GROUPER_GAIN);
    uint32_t grp_ono = aes70_object_ono(grp);
    uint32_t c1 = aes70_object_ono(g1), c2 = aes70_object_ono(g2);
    conn_set((aes70_device_t *)dev, 0, false);

    uint8_t p[64], rp[64]; size_t rl; uint8_t st; ocp1_wr_t w; ocp1_rd_t r;

    /* AddGroup("G1") -> (groupIndex u16, proxyONo u32). */
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_string(&w, "G1");
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 1, p, w.off, rp, &rl);
    CHECK(st == AES70_OK, "AddGroup status = %u", st);
    ocp1_rd_init(&r, rp, rl);
    uint16_t gi = ocp1_rd_u16(&r); uint32_t proxy = ocp1_rd_u32(&r);
    CHECK(gi == 1, "group index = %u", gi);
    CHECK(proxy != 0, "proxy ONo non-zero");

    /* AddCitizen(c1), AddCitizen(c2): OcaGrouperCitizen{idx, OPath{hostBlob, ONo}, online}. */
    uint16_t ci1 = 0, ci2 = 0;
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, 0); ocp1_wr_u16(&w, 0); ocp1_wr_u32(&w, c1); ocp1_wr_u8(&w, 1);
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 5, p, w.off, rp, &rl);
    CHECK(st == AES70_OK, "AddCitizen1 = %u", st); ci1 = resp_u16(rp);
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, 0); ocp1_wr_u16(&w, 0); ocp1_wr_u32(&w, c2); ocp1_wr_u8(&w, 1);
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 5, p, w.off, rp, &rl);
    CHECK(st == AES70_OK, "AddCitizen2 = %u", st); ci2 = resp_u16(rp);

    /* Adding a citizen of the wrong class is rejected. */
    aes70_object_handle_t mute = aes70_mute_create(dev, NULL, "M", false);
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, 0); ocp1_wr_u16(&w, 0);
    ocp1_wr_u32(&w, aes70_object_ono(mute)); ocp1_wr_u8(&w, 1);
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 5, p, w.off, NULL, NULL);
    CHECK(st == AES70_PARAMETER_ERROR, "wrong-class citizen rejected = %u", st);

    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 3, NULL, 0, rp, &rl);   /* GetGroupCount */
    CHECK(st == AES70_OK && resp_u16(rp) == 1, "group count");
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 7, NULL, 0, rp, &rl);   /* GetCitizenCount */
    CHECK(st == AES70_OK && resp_u16(rp) == 2, "citizen count = %u", resp_u16(rp));

    /* Enroll both citizens in the group. */
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, gi); ocp1_wr_u16(&w, ci1); ocp1_wr_u8(&w, 1);
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 10, p, w.off, NULL, NULL);
    CHECK(st == AES70_OK, "SetEnrollment1 = %u", st);
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, gi); ocp1_wr_u16(&w, ci2); ocp1_wr_u8(&w, 1);
    route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 10, p, w.off, NULL, NULL);

    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, gi); ocp1_wr_u16(&w, ci1);  /* GetEnrollment */
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 9, p, w.off, rp, &rl);
    CHECK(st == AES70_OK && rp[0] == 1, "GetEnrollment true");

    /* Fan-out: writing the group's proxy gain drives both enrolled citizens. */
    f32_to_params(-10.f, p);
    st = route_cmd((aes70_device_t *)dev, 0, proxy, 4, 2, p, 4, NULL, NULL);
    CHECK(st == AES70_OK, "proxy SetGain = %u", st);
    CHECK(fabsf(aes70_gain_get(g1) + 10.f) < 1e-3f, "citizen1 followed (%.2f)", aes70_gain_get(g1));
    CHECK(fabsf(aes70_gain_get(g2) + 10.f) < 1e-3f, "citizen2 followed (%.2f)", aes70_gain_get(g2));

    /* Un-enroll citizen 2; it should stop following. */
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_u16(&w, gi); ocp1_wr_u16(&w, ci2); ocp1_wr_u8(&w, 0);
    route_cmd((aes70_device_t *)dev, 0, grp_ono, 3, 10, p, w.off, NULL, NULL);
    f32_to_params(-20.f, p);
    route_cmd((aes70_device_t *)dev, 0, proxy, 4, 2, p, 4, NULL, NULL);
    CHECK(fabsf(aes70_gain_get(g1) + 20.f) < 1e-3f, "c1 follows after unenroll (%.2f)", aes70_gain_get(g1));
    CHECK(fabsf(aes70_gain_get(g2) + 10.f) < 1e-3f, "c2 frozen after unenroll (%.2f)", aes70_gain_get(g2));

    /* OcaAgent base methods (SetLabel/GetLabel at level 2). */
    ocp1_wr_init(&w, p, sizeof p); ocp1_wr_string(&w, "MyGroup");
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 2, 2, p, w.off, NULL, NULL);
    CHECK(st == AES70_OK, "agent SetLabel = %u", st);
    st = route_cmd((aes70_device_t *)dev, 0, grp_ono, 2, 1, NULL, 0, rp, &rl);
    CHECK(st == AES70_OK && resp_u16(rp) == 7, "agent GetLabel returns 'MyGroup'");

    aes70_device_stop(dev);
}

int main(void)
{
    printf("AES70 host unit tests\n");
    test_codec();
    test_method_is_write();
    test_router_and_security();
    test_locking();
    test_bad_ono();
    test_grouper();
    printf("%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
