//
// SSD1322_priv.h
//
//  Created on: Mar 17, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_SSD1322_PRIV_H_
#define SRC_EVDRIVERS_SSD1322_PRIV_H_

#include <cstdint>

namespace ssd1322 {
const uint8_t enable_grayscale_table = 0x00;
const uint8_t set_column_address = 0x15;
const uint8_t write_ram_command = 0x5C;
const uint8_t read_ram_command = 0x5D;
const uint8_t set_row_address = 0x75;
const uint8_t set_remap_and_dual_com_line_mode = 0xA0;
const uint8_t set_display_start_line = 0xA1;
const uint8_t set_display_offset = 0xA2;
const uint8_t set_display_mode_all_off = 0xA4;
const uint8_t set_display_mode_all_on = 0xA5;
const uint8_t set_display_mode_normal = 0xA6;
const uint8_t set_display_mode_inverse = 0xA7;
const uint8_t enable_partial_display = 0xA8;
const uint8_t exit_partial_display = 0xA9;
const uint8_t set_function_selection = 0xAB;
const uint8_t set_display_off = 0xAE;
const uint8_t set_display_on = 0xAF;
const uint8_t set_phase_length = 0xB1;
const uint8_t set_divider_and_frequency = 0xB3;
const uint8_t display_enhancement_a = 0xB4;
const uint8_t set_gpio = 0xB5;
const uint8_t set_second_precharge_period = 0xB6;
const uint8_t set_grayscale_table = 0xB8;
const uint8_t select_default_linear_grayscale_table = 0xB9;
const uint8_t set_precharge_voltage = 0xBB;
const uint8_t set_vcomh_voltage = 0xBE;
const uint8_t set_contrast_current = 0xC1;
const uint8_t master_current_control = 0xC7;
const uint8_t set_multiplex_ratio = 0xCA;
const uint8_t display_enhancement_b = 0xD1;
const uint8_t set_command_lock = 0xFD;
} // namespace ssd1322

#endif // SRC_EVDRIVERS_SSD1322_PRIV_H_
