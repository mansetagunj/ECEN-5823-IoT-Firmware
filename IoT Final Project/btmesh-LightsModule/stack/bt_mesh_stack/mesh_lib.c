#include <stdint.h>
#include <stdlib.h>

/* BG stack headers */
#include "bg_types.h"

#if (!defined(MESH_LIB_NATIVE) && !defined(MESH_LIB_HOST)) \
  || (defined(MESH_LIB_NATIVE) && defined(MESH_LIB_HOST))
#error "You must define either MESH_LIB_NATIVE or MESH_LIB_HOST, and not both!"
#endif

/* Select BGAPI flavor */
#if defined(MESH_LIB_HOST)
#include "gecko_bglib.h"
#include "gecko_bgapi_mesh_generic_client_host.h"
#include "gecko_bgapi_mesh_generic_server_host.h"
#else /* MESH_LIB_NATIVE */
#include "gecko_bgapi_mesh_generic_client_native.h"
#include "gecko_bgapi_mesh_generic_server_native.h"
#endif /* MESH_LIB_NATIVE */

#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

uint32_t mesh_lib_transition_time_to_ms(uint8_t t)
{
  uint32_t res_ms[4] = { 100, 1000, 10000, 600000 };
  uint32_t unit = (t >> 6) & 0x03;
  uint32_t count = t & 0x3f;
  return res_ms[unit] * count;
}

static int16_t int16_from_buf(const uint8_t *ptr)
{
  return ((int16_t)ptr[0]) | ((int16_t)ptr[1] << 8);
}

static void int16_to_buf(uint8_t *ptr, int16_t n)
{
  ptr[0] = n & 0xff;
  ptr[1] = (n >> 8) & 0xff;
}

static uint16_t uint16_from_buf(const uint8_t *ptr)
{
  return ((uint16_t)ptr[0]) | ((uint16_t)ptr[1] << 8);
}

static void uint16_to_buf(uint8_t *ptr, uint16_t n)
{
  ptr[0] = n & 0xff;
  ptr[1] = (n >> 8) & 0xff;
}

static int32_t int32_from_buf(const uint8_t *ptr)
{
  return
    ((int16_t)ptr[0])
    | ((int16_t)ptr[1] << 8)
    | ((int16_t)ptr[2] << 16)
    | ((int16_t)ptr[3] << 24)
  ;
}

static void int32_to_buf(uint8_t *ptr, int16_t n)
{
  ptr[0] = n & 0xff;
  ptr[1] = (n >> 8) & 0xff;
  ptr[2] = (n >> 16) & 0xff;
  ptr[3] = (n >> 24) & 0xff;
}

errorcode_t serialize_request(const struct mesh_generic_request *req,
                              uint8_t *msg_buf,
                              size_t msg_len,
                              size_t *msg_used)
{
  size_t msg_off = 0;

  switch (req->kind) {
    case mesh_generic_request_on_off:
      if (msg_len < 1) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = req->on_off;
      *msg_used = msg_off;
      break;

    case mesh_generic_request_on_power_up:
      if (msg_len < 1) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = req->on_power_up;
      *msg_used = msg_off;
      break;

    case mesh_generic_request_transition_time:
      if (msg_len < 1) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = req->transition_time;
      *msg_used = msg_off;
      break;

    case mesh_generic_request_level:
    case mesh_generic_request_level_move:
    case mesh_generic_request_level_halt:
      if (msg_len < 2) {
        return bg_err_invalid_param;
      }
      int16_to_buf(&msg_buf[msg_off], req->level);
      msg_off += 2;
      *msg_used = msg_off;
      break;

    case mesh_generic_request_level_delta:
      if (msg_len < 4) {
        return bg_err_invalid_param;
      }
      int32_to_buf(&msg_buf[msg_off], req->delta);
      msg_off += 4;
      *msg_used = msg_off;
      break;

    case mesh_generic_request_power_level:
    case mesh_generic_request_power_level_default:
    case mesh_generic_request_power_level_range:
    case mesh_generic_request_location_global:
    case mesh_generic_request_location_local:
    case mesh_generic_request_property_user:
    case mesh_generic_request_property_admin:
    case mesh_generic_request_property_manuf:
      return bg_err_not_implemented;

    case mesh_lighting_request_lightness_actual:
    case mesh_lighting_request_lightness_linear:
    case mesh_lighting_request_lightness_default:
      if (msg_len < 2) {
        return bg_err_invalid_param;
      }
      uint16_to_buf(&msg_buf[msg_off], req->lightness);
      msg_off += 2;
      *msg_used = msg_off;
      break;

    case mesh_lighting_request_lightness_range:
      if (msg_len < 4) {
        return bg_err_invalid_param;
      }
      uint16_to_buf(&msg_buf[msg_off], req->lightness_range.min);
      msg_off += 2;
      uint16_to_buf(&msg_buf[msg_off], req->lightness_range.max);
      msg_off += 2;
      *msg_used = msg_off;
      break;

    default:
      return bg_err_invalid_param;
  }

  return bg_err_success;
}

errorcode_t deserialize_request(struct mesh_generic_request *req,
                                mesh_generic_request_t kind,
                                const uint8_t *msg_buf,
                                size_t msg_len)
{
  size_t msg_off = 0;

  switch (kind) {
    case mesh_generic_request_on_off:
      if (msg_len - msg_off != 1) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->on_off = msg_buf[msg_off];
      break;

    case mesh_generic_request_on_power_up:
      if (msg_len - msg_off != 1) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->on_power_up = msg_buf[msg_off];
      break;

    case mesh_generic_request_transition_time:
      if (msg_len - msg_off != 1) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->transition_time = msg_buf[msg_off];
      break;

    case mesh_generic_request_level:
    case mesh_generic_request_level_move:
    case mesh_generic_request_level_halt:
      if (msg_len - msg_off != 2) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->level = int16_from_buf(&msg_buf[msg_off]);
      break;

    case mesh_generic_request_level_delta:
      if (msg_len - msg_off != 4) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->level = int32_from_buf(&msg_buf[msg_off]);
      break;

    case mesh_generic_request_power_level:
    case mesh_generic_request_power_level_default:
    case mesh_generic_request_power_level_range:
    case mesh_generic_request_location_global:
    case mesh_generic_request_location_local:
    case mesh_generic_request_property_user:
    case mesh_generic_request_property_admin:
    case mesh_generic_request_property_manuf:
      return bg_err_not_implemented;

    case mesh_lighting_request_lightness_actual:
    case mesh_lighting_request_lightness_linear:
    case mesh_lighting_request_lightness_default:
      if (msg_len - msg_off != 2) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->lightness = uint16_from_buf(&msg_buf[msg_off]);
      break;

    case mesh_lighting_request_lightness_range:
      if (msg_len - msg_off != 4) {
        return bg_err_invalid_param;
      }
      req->kind = kind;
      req->lightness_range.min = uint16_from_buf(&msg_buf[msg_off]);
      req->lightness_range.max = uint16_from_buf(&msg_buf[msg_off + 2]);
      break;

    default:
      return bg_err_invalid_param;
  }

  return bg_err_success;
}

errorcode_t serialize_state(const struct mesh_generic_state *current,
                            const struct mesh_generic_state *target,
                            uint8_t *msg_buf,
                            size_t msg_len,
                            size_t *msg_used)
{
  size_t msg_off = 0;

  switch (current->kind) {
    case mesh_generic_state_on_off:
      if (msg_len < (target ? 2 : 1)) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = current->on_off.on;
      if (target) {
        msg_buf[msg_off++] = target->on_off.on;
      }
      *msg_used = msg_off;
      break;

    case mesh_generic_state_on_power_up:
      if (msg_len < 1) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = current->on_power_up.on_power_up;
      *msg_used = msg_off;
      break;

    case mesh_generic_state_transition_time:
      if (msg_len < 1) {
        return bg_err_invalid_param;
      }
      msg_buf[msg_off++] = current->transition_time.time;
      *msg_used = msg_off;
      break;

    case mesh_generic_state_level:
      if (msg_len < (target ? 4 : 2)) {
        return bg_err_invalid_param;
      }
      int16_to_buf(&msg_buf[msg_off], current->level.level);
      msg_off += 2;
      if (target) {
        int16_to_buf(&msg_buf[msg_off], target->level.level);
        msg_off += 2;
      }
      *msg_used = msg_off;
      break;

    case mesh_generic_state_power_level:
    case mesh_generic_state_power_level_last:
    case mesh_generic_state_power_level_default:
    case mesh_generic_state_power_level_range:
    case mesh_generic_state_battery:
    case mesh_generic_state_location_global:
    case mesh_generic_state_location_local:
    case mesh_generic_state_property_user:
    case mesh_generic_state_property_admin:
    case mesh_generic_state_property_manuf:
    case mesh_generic_state_property_list_user:
    case mesh_generic_state_property_list_admin:
    case mesh_generic_state_property_list_manuf:
    case mesh_generic_state_property_list_client:
      return bg_err_not_implemented;

    case mesh_lighting_state_lightness_actual:
    case mesh_lighting_state_lightness_linear:
      if (msg_len < (target ? 4 : 2)) {
        return bg_err_invalid_param;
      }
      uint16_to_buf(&msg_buf[msg_off], current->lightness.level);
      msg_off += 2;
      if (target) {
        uint16_to_buf(&msg_buf[msg_off], target->lightness.level);
        msg_off += 2;
      }
      *msg_used = msg_off;
      break;

    case mesh_lighting_state_lightness_last:
    case mesh_lighting_state_lightness_default:
      if (msg_len < 2) {
        return bg_err_invalid_param;
      }
      uint16_to_buf(&msg_buf[msg_off], current->lightness.level);
      msg_off += 2;
      *msg_used = msg_off;
      break;

    case mesh_lighting_state_lightness_range:
      if (msg_len < 4) {
        return bg_err_invalid_param;
      }
      uint16_to_buf(&msg_buf[msg_off], current->lightness_range.min);
      msg_off += 2;
      uint16_to_buf(&msg_buf[msg_off], current->lightness_range.max);
      msg_off += 2;
      *msg_used = msg_off;
      break;

    case mesh_generic_state_last:
      return bg_err_invalid_param;
  }

  return bg_err_success;
}

errorcode_t deserialize_state(struct mesh_generic_state *current,
                              struct mesh_generic_state *target,
                              int *has_target,
                              mesh_generic_state_t kind,
                              const uint8_t *msg_buf,
                              size_t msg_len)
{
  size_t msg_off = 0;

  switch (kind) {
    case mesh_generic_state_on_off:
      if (msg_len - msg_off == 1) {
        current->kind = kind;
        current->on_off.on = msg_buf[msg_off++];
        *has_target = 0;
      } else if (msg_len - msg_off == 2) {
        current->kind = kind;
        current->on_off.on = msg_buf[msg_off++];
        target->kind = kind;
        target->on_off.on = msg_buf[msg_off++];
        *has_target = 1;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_generic_state_on_power_up:
      if (msg_len - msg_off == 1) {
        current->kind = kind;
        current->on_power_up.on_power_up = msg_buf[msg_off++];
        *has_target = 0;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_generic_state_transition_time:
      if (msg_len - msg_off == 1) {
        current->kind = kind;
        current->transition_time.time = msg_buf[msg_off++];
        *has_target = 0;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_generic_state_level:
      if (msg_len - msg_off == 2) {
        current->kind = kind;
        current->level.level = int16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 0;
      } else if (msg_len - msg_off == 4) {
        current->kind = kind;
        current->level.level = int16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        target->kind = kind;
        target->level.level = int16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 1;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_generic_state_power_level:
    case mesh_generic_state_power_level_last:
    case mesh_generic_state_power_level_default:
    case mesh_generic_state_power_level_range:
    case mesh_generic_state_battery:
    case mesh_generic_state_location_global:
    case mesh_generic_state_location_local:
    case mesh_generic_state_property_user:
    case mesh_generic_state_property_admin:
    case mesh_generic_state_property_manuf:
    case mesh_generic_state_property_list_user:
    case mesh_generic_state_property_list_admin:
    case mesh_generic_state_property_list_manuf:
    case mesh_generic_state_property_list_client:
      return bg_err_not_implemented;

    case mesh_lighting_state_lightness_actual:
    case mesh_lighting_state_lightness_linear:
      if (msg_len - msg_off == 2) {
        current->kind = kind;
        current->lightness.level = uint16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 0;
      } else if (msg_len - msg_off == 4) {
        current->kind = kind;
        current->lightness.level = int16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        target->kind = kind;
        target->lightness.level = int16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 1;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_lighting_state_lightness_last:
    case mesh_lighting_state_lightness_default:
      if (msg_len - msg_off == 2) {
        current->kind = kind;
        current->lightness.level = uint16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 0;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_lighting_state_lightness_range:
      if (msg_len - msg_off == 4) {
        current->kind = kind;
        current->lightness_range.min = uint16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        current->lightness_range.max = uint16_from_buf(&msg_buf[msg_off]);
        msg_off += 2;
        *has_target = 0;
      } else {
        return bg_err_invalid_param;
      }
      break;

    case mesh_generic_state_last:
      return bg_err_invalid_param;
  }

  return bg_err_invalid_param;
}

struct reg {
  uint16_t model_id;
  uint16_t elem_index;
  union {
    struct {
      mesh_lib_generic_server_client_request_cb client_request_cb;
      mesh_lib_generic_server_change_cb state_changed_cb;
    } server;
    struct {
      mesh_lib_generic_client_server_response_cb server_response_cb;
    } client;
  };
};

static struct reg *reg = NULL;
static size_t regs = 0;

static void *(*lib_malloc_fn)(size_t) = NULL;
static void (*lib_free_fn)(void *) = NULL;

static struct reg *find_reg(uint16_t model_id,
                            uint16_t elem_index)
{
  size_t r;
  for (r = 0; r < regs; r++) {
    if (reg[r].model_id == model_id && reg[r].elem_index == elem_index) {
      return &reg[r];
    }
  }
  return NULL;
}

static struct reg *find_free(void)
{
  size_t r;
  for (r = 0; r < regs; r++) {
    if (reg[r].model_id == 0x0000 && reg[r].elem_index == 0x0000) {
      return &reg[r];
    }
  }
  return NULL;
}

errorcode_t mesh_lib_init(void *(*malloc_fn)(size_t),
                          void (*free_fn)(void *),
                          size_t generic_models)
{
  lib_malloc_fn = malloc_fn;
  lib_free_fn = free_fn;

  if (generic_models) {
    reg = (lib_malloc_fn)(generic_models * sizeof(struct reg));
    if (!reg) {
      return bg_err_out_of_memory;
    }
    memset(reg, 0, generic_models * sizeof(struct reg));
    regs = generic_models;
  }

  return bg_err_success;
}

void mesh_lib_deinit(void)
{
  if (reg) {
    (lib_free_fn)(reg);
    reg = NULL;
    regs = 0;
  }
}

errorcode_t
mesh_lib_generic_server_register_handler(uint16_t model_id,
                                         uint16_t elem_index,
                                         mesh_lib_generic_server_client_request_cb cb,
                                         mesh_lib_generic_server_change_cb ch)
{
  struct reg *reg = NULL;

  reg = find_reg(model_id, elem_index);
  if (reg) {
    return bg_err_wrong_state; // already exists
  }

  reg = find_free();
  if (!reg) {
    return bg_err_out_of_memory;
  }

  reg->model_id = model_id;
  reg->elem_index = elem_index;
  reg->server.client_request_cb = cb;
  reg->server.state_changed_cb = ch;
  return bg_err_success;
}

errorcode_t
mesh_lib_generic_client_register_handler(uint16_t model_id,
                                         uint16_t elem_index,
                                         mesh_lib_generic_client_server_response_cb cb)
{
  struct reg *reg = NULL;

  reg = find_reg(model_id, elem_index);
  if (reg) {
    return bg_err_wrong_state; // already exists
  }

  reg = find_free();
  if (!reg) {
    return bg_err_out_of_memory;
  }

  reg->model_id = model_id;
  reg->elem_index = elem_index;
  reg->client.server_response_cb = cb;
  return bg_err_success;
}

void mesh_lib_generic_server_event_handler(struct gecko_bgapi_mesh_generic_server_cmd_packet *evt)
{
  struct gecko_msg_mesh_generic_server_client_request_evt_t *req = NULL;
  struct gecko_msg_mesh_generic_server_state_changed_evt_t *chg = NULL;
  struct mesh_generic_request request;
  struct mesh_generic_state current;
  struct mesh_generic_state target;
  int has_target;
  struct reg *reg;

  if (!evt) {
    return;
  }

  switch (BGLIB_MSG_ID(evt->header)) {
    case gecko_evt_mesh_generic_server_client_request_id:
      req = &(evt->data.evt_mesh_generic_server_client_request);
      reg = find_reg(req->model_id, req->elem_index);
      if (reg) {
        if (deserialize_request(&request,
                                req->type,
                                req->parameters.data,
                                req->parameters.len) == bg_err_success) {
          (reg->server.client_request_cb)(req->model_id,
                                          req->elem_index,
                                          req->client_address,
                                          req->server_address,
                                          req->appkey_index,
                                          &request,
                                          req->transition,
                                          req->delay,
                                          req->flags);
        }
      }
      break;
    case gecko_evt_mesh_generic_server_state_changed_id:
      chg = &(evt->data.evt_mesh_generic_server_state_changed);
      reg = find_reg(req->model_id, req->elem_index);
      if (reg) {
        if (deserialize_state(&current,
                              &target,
                              &has_target,
                              chg->type,
                              chg->parameters.data,
                              chg->parameters.len) == bg_err_success) {
          (reg->server.state_changed_cb)(req->model_id,
                                         req->elem_index,
                                         &current,
                                         has_target ? &target : NULL,
                                         chg->remaining);
        }
      }
      break;
  }
}

void mesh_lib_generic_client_event_handler(struct gecko_bgapi_mesh_generic_client_cmd_packet *evt)
{
  struct gecko_msg_mesh_generic_client_server_status_evt_t *res = NULL;
  struct mesh_generic_state current;
  struct mesh_generic_state target;
  int has_target;
  struct reg *reg;

  if (!evt) {
    return;
  }

  switch (BGLIB_MSG_ID(evt->header)) {
    case gecko_evt_mesh_generic_client_server_status_id:
      res = &(evt->data.evt_mesh_generic_client_server_status);
      reg = find_reg(res->model_id, res->elem_index);
      if (reg) {
        if (deserialize_state(&current,
                              &target,
                              &has_target,
                              res->type,
                              res->parameters.data,
                              res->parameters.len) == bg_err_success) {
          (reg->client.server_response_cb)(res->model_id,
                                           res->elem_index,
                                           res->client_address,
                                           res->server_address,
                                           &current,
                                           has_target ? &target : NULL,
                                           res->remaining,
                                           res->flags);
        }
      }
      break;
  }
}

errorcode_t
mesh_lib_generic_server_response(uint16_t model_id,
                                 uint16_t element_index,
                                 uint16_t client_addr,
                                 uint16_t appkey_index,
                                 const struct mesh_generic_state *current,
                                 const struct mesh_generic_state *target,
                                 uint32_t remaining_ms,
                                 uint8_t response_flags)
{
  errorcode_t e;
  uint8_t buf[10];
  size_t len;

  e = serialize_state(current, target, buf, sizeof(buf), &len);
  if (e != bg_err_success) {
    return e;
  }
  return gecko_cmd_mesh_generic_server_response(model_id,
                                                element_index,
                                                client_addr,
                                                appkey_index,
                                                remaining_ms,
                                                response_flags,
                                                current->kind,
                                                len,
                                                buf)->result;
}

errorcode_t
mesh_lib_generic_server_update(uint16_t model_id,
                               uint16_t element_index,
                               const struct mesh_generic_state *current,
                               const struct mesh_generic_state *target,
                               uint32_t remaining_ms)
{
  errorcode_t e;
  uint8_t buf[10];
  size_t len;

  e = serialize_state(current, target, buf, sizeof(buf), &len);
  if (e != bg_err_success) {
    return e;
  }
  return gecko_cmd_mesh_generic_server_update(model_id,
                                              element_index,
                                              remaining_ms,
                                              current->kind,
                                              len,
                                              buf)->result;
}

errorcode_t
mesh_lib_generic_server_publish(uint16_t model_id,
                                uint16_t element_index,
                                mesh_generic_state_t kind)
{
  return gecko_cmd_mesh_generic_server_publish(model_id,
                                               element_index,
                                               kind)->result;
}

errorcode_t mesh_lib_generic_client_get(uint16_t model_id,
                                        uint16_t element_index,
                                        uint16_t server_addr,
                                        uint16_t appkey_index,
                                        mesh_generic_state_t kind)
{
  return gecko_cmd_mesh_generic_client_get(model_id,
                                           element_index,
                                           server_addr,
                                           appkey_index,
                                           kind)->result;
}

errorcode_t mesh_lib_generic_client_set(uint16_t model_id,
                                        uint16_t element_index,
                                        uint16_t server_addr,
                                        uint16_t appkey_index,
                                        uint8_t transaction_id,
                                        const struct mesh_generic_request *request,
                                        uint32_t transition_ms,
                                        uint16_t delay_ms,
                                        uint8_t flags)
{
  errorcode_t e;
  uint8_t buf[10];
  size_t len;

  e = serialize_request(request, buf, sizeof(buf), &len);
  if (e != bg_err_success) {
    return e;
  }
  return gecko_cmd_mesh_generic_client_set(model_id,
                                           element_index,
                                           server_addr,
                                           appkey_index,
                                           transaction_id,
                                           transition_ms,
                                           delay_ms,
                                           flags,
                                           request->kind,
                                           len,
                                           buf)->result;
}

errorcode_t
mesh_lib_generic_client_publish(uint16_t model_id,
                                uint16_t element_index,
                                uint16_t appkey_index,
                                uint8_t transaction_id,
                                const struct mesh_generic_request *request,
                                uint32_t transition_ms,
                                uint16_t delay_ms,
                                uint8_t request_flags)
{
  errorcode_t e;
  uint8_t buf[10];
  size_t len;

  e = serialize_request(request, buf, sizeof(buf), &len);
  if (e != bg_err_success) {
    return e;
  }
  return gecko_cmd_mesh_generic_client_publish(model_id,
                                               element_index,
                                               transaction_id,
                                               transition_ms,
                                               delay_ms,
                                               request_flags,
                                               request->kind,
                                               len,
                                               buf)->result;
}
