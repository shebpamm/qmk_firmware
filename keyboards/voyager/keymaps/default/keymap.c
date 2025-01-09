#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_120_128_190,
  HSV_74_84_147,
  HSV_215_108_223,
  ST_MACRO_0,
  NULL_A,
  NULL_D,
  AUTO_CLICK,
};

const uint16_t null_to_standard[] = {
  [NULL_A] = KC_A,
  [NULL_D] = KC_D,
};

bool autoclicker_enabled = false;
uint16_t autoclicker_timer;

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_F20,         AUTO_CLICK, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_TRANSPARENT,
    MO(5),          MT(MOD_LGUI, KC_A),MT(MOD_LALT, KC_S),MT(MOD_LCTL, KC_D),MT(MOD_LSFT, KC_F),KC_G,                                           KC_H,           MT(MOD_LSFT, KC_J),MT(MOD_LCTL, KC_K),MT(MOD_LALT, KC_L),MT(MOD_LGUI, KC_SCLN),KC_TRANSPARENT,
    KC_TRANSPARENT, KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_TRANSPARENT, KC_TRANSPARENT,
                                                    LT(3,KC_SPACE), LT(2,KC_ESCAPE),                                LT(4,KC_BSPC),  LT(1,KC_ENTER)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_GRAVE,       KC_LPRN,        KC_RPRN,        KC_SCLN,        KC_COMMA,                                       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_EXLM,        KC_LCBR,        KC_QUOTE,       KC_DQUO,        KC_RCBR,        KC_QUES,                                        KC_SCLN,        KC_PERC,        KC_COLN,        KC_DOT,         KC_TRANSPARENT, KC_TRANSPARENT,
    KC_HASH,        KC_CIRC,        KC_EQUAL,       KC_UNDS,        KC_DLR,         KC_ASTR,                                        KC_SLASH,       TD(DANCE_0),    MT(MOD_LCTL, KC_LBRC),MT(MOD_LALT, KC_RBRC),TD(DANCE_1),    KC_TRANSPARENT,
    KC_TILD,        KC_LABK,        KC_PIPE,        KC_MINUS,       KC_RABK,        KC_SLASH,                                       KC_AT,          KC_AMPR,        KC_LBRC,        KC_RBRC,        KC_PLUS,        KC_BSLS,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_F10,         KC_F11,         KC_F12,         KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_F7,          KC_F8,          KC_F9,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_7,           KC_8,           KC_9,           KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, MT(MOD_LALT, KC_F4),MT(MOD_LCTL, KC_F5),MT(MOD_LSFT, KC_F6),KC_TRANSPARENT,                                 KC_TRANSPARENT, MT(MOD_LSFT, KC_4),MT(MOD_LCTL, KC_5),MT(MOD_LALT, KC_6),KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 LT(9,KC_BSPC),  KC_0
  ),
  [3] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_MUTE,  KC_AUDIO_VOL_UP,KC_TRANSPARENT,                                 KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_TRANSPARENT, KC_TRANSPARENT,
    KC_DELETE,      KC_TRANSPARENT, MT(MOD_LALT, KC_MEDIA_PREV_TRACK),MT(MOD_LCTL, KC_MEDIA_PLAY_PAUSE),MT(MOD_LSFT, KC_MEDIA_NEXT_TRACK),KC_TRANSPARENT,                                 KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_TRANSPARENT, TO(8),
    KC_TRANSPARENT, RGB_HUD,        RGB_HUI,        HSV_120_128_190,HSV_74_84_147,  HSV_215_108_223,                                KC_TRANSPARENT, LCTL(LSFT(KC_TAB)),LCTL(KC_TAB),   ST_MACRO_0,     TO(6),          TO(7),
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RALT(KC_O),     KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, RALT(KC_A),     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_UP,    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_MS_BTN1,     KC_MS_UP,       KC_MS_BTN2,     KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_MS_WH_LEFT,  KC_MS_ACCEL0,   KC_MS_ACCEL1,   KC_MS_ACCEL2,   KC_MS_WH_RIGHT,                                 KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_DOWN,  KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [6] = LAYOUT_voyager(
    KC_F13,         KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_4,                                           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_ESCAPE,      KC_TRANSPARENT, KC_Q,           KC_W,           KC_E,           KC_R,                                           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_LEFT_SHIFT,  KC_LEFT_ALT,    NULL_A,         KC_S,           NULL_D,         KC_F,                                           KC_TRANSPARENT, KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_TRANSPARENT,
    KC_LEFT_CTRL,   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(0),
                                                    KC_SPACE,       KC_TAB,                                         KC_BSPC,        KC_ENTER
  ),
  [7] = LAYOUT_voyager(
    KC_F18,         KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_TRANSPARENT,
    KC_ESCAPE,      KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_LEFT_SHIFT,  KC_A,           KC_S,           KC_D,           KC_F,           KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_F15,         KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(0),
                                                    KC_SPACE,       KC_F20,                                         KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [8] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_LEFT_SHIFT,  KC_A,           KC_S,           KC_D,           KC_F,           KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_LEFT_ALT,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(0),
                                                    KC_SPACE,       KC_LEFT_CTRL,                                   KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [9] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_KP_7,        KC_KP_8,        KC_KP_9,        KC_KP_SLASH,    KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_KP_4,        KC_KP_5,        KC_KP_6,        KC_KP_COMMA,    KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_KP_1,        KC_KP_2,        KC_KP_3,        KC_KP_DOT,      KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_KP_0
  ),
};

const uint16_t PROGMEM combo0[] = { LT(3,KC_SPACE), LT(1,KC_ENTER), COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, TT(3)),
};


extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215}, {82,131,215} },

    [6] = { {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {82,108,227}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {82,108,227}, {82,108,227}, {82,108,227}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181}, {139,205,181} },

    [7] = { {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227}, {82,108,227} },

    [8] = { {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221}, {31,226,221} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 6:
      set_layer_color(6);
      break;
    case 7:
      set_layer_color(7);
      break;
    case 8:
      set_layer_color(8);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_TAP(X_D)));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_120_128_190:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(120,128,190);
      }
      return false;
    case HSV_74_84_147:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(74,84,147);
      }
      return false;
    case HSV_215_108_223:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(215,108,223);
      }
      return false;
    case AUTO_CLICK:
        if (record->event.pressed) {
            autoclicker_enabled = true;
            autoclicker_timer = timer_read();
        } else {
            autoclicker_enabled = false;
        }
        return false;
    case NULL_A:
    case NULL_D:
      if (record->event.pressed) {
        // Release all other custom WASD keys
        for (uint16_t i = NULL_A; i <= NULL_D; i++) {
            if (i != keycode) {
                unregister_code(null_to_standard[i]);
            }
        }

        // Register the currently pressed custom key
        register_code(null_to_standard[keycode]);
      } else {
        // Unregister the currently released custom key
        unregister_code(null_to_standard[keycode]);
      }
    return false; // We handled this key, return false to stop further processing

  }
  return true;
}

void matrix_scan_user(void) {
    if (autoclicker_enabled) {
        if (timer_elapsed(autoclicker_timer) > 50) {
            tap_code(KC_MS_BTN1);
            autoclicker_timer = timer_read();
        }
    }
}

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
    }
    if(state->count > 3) {
        tap_code16(KC_LPRN);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_LPRN); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: register_code16(KC_LPRN); register_code16(KC_LPRN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LPRN); register_code16(KC_LPRN);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_LPRN); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(KC_LPRN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LPRN); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_RPRN);
        tap_code16(KC_RPRN);
        tap_code16(KC_RPRN);
    }
    if(state->count > 3) {
        tap_code16(KC_RPRN);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_RPRN); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: register_code16(KC_RPRN); register_code16(KC_RPRN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_RPRN); register_code16(KC_RPRN);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_RPRN); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: unregister_code16(KC_RPRN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_RPRN); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
};
