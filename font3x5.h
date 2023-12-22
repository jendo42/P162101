#pragma once

const uint8_t font3x5_bitmap[] = {
 // ' '
0x17,  // '!'
0x3, 0x3,  // '"'
0x1F, 0xA, 0x1F,  // '#'
0xE, 0x1B, 0xA,  // '$'
0x19, 0x4, 0x13,  // '%'
0x1B, 0x15, 0x1F,  // '&'
0x3,  // '''
0xE, 0x11,  // '('
0x11, 0xE,  // ')'
0x2, 0x7, 0x2,  // '*'
0x4, 0xE, 0x4,  // '+'
0x10, 0x8,  // ','
0x4, 0x4, 0x4,  // '-'
0x10,  // '.'
0x18, 0x4, 0x3,  // '/'
0x1F, 0x11, 0x1F,  // '0'
0x2, 0x1F,  // '1'
0x12, 0x19, 0x16,  // '2'
0x11, 0x15, 0xA,  // '3'
0x7, 0x4, 0x1F,  // '4'
0x17, 0x15, 0xD,  // '5'
0x1F, 0x15, 0x1D,  // '6'
0x1, 0x1D, 0x3,  // '7'
0x1B, 0x15, 0x1B,  // '8'
0x17, 0x15, 0x1F,  // '9'
0xA,  // ':'
0x10, 0xA,  // ';'
0x4, 0xA, 0xA,  // '<'
0xA, 0xA, 0xA,  // '='
0xA, 0xA, 0x4,  // '>'
0x1, 0x15, 0x3,  // '?'
0x1F, 0x11, 0x17,  // '@'
0x1F, 0x5, 0x1F,  // 'A'
0x1F, 0x15, 0x1B,  // 'B'
0x1F, 0x11, 0x11,  // 'C'
0x1F, 0x11, 0xE,  // 'D'
0x1F, 0x15, 0x11,  // 'E'
0x1F, 0x5, 0x1,  // 'F'
0x1F, 0x11, 0x1D,  // 'G'
0x1F, 0x4, 0x1F,  // 'H'
0x1D,  // 'I'
0x18, 0x10, 0x1F,  // 'J'
0x1F, 0x4, 0x1B,  // 'K'
0x1F, 0x10, 0x10,  // 'L'
0x1F, 0x2, 0x1F,  // 'M'
0x1F, 0x6, 0x1F,  // 'N'
0x1F, 0x11, 0x1F,  // 'O'
0x1F, 0x5, 0x7,  // 'P'
0xF, 0x9, 0x1F,  // 'Q'
0x1F, 0x5, 0x1B,  // 'R'
0x17, 0x15, 0x1D,  // 'S'
0x1, 0x1F, 0x1,  // 'T'
0x1F, 0x10, 0x1F,  // 'U'
0xF, 0x10, 0xF,  // 'V'
0x1F, 0x8, 0x1F,  // 'W'
0x1B, 0x4, 0x1B,  // 'X'
0x3, 0x1C, 0x3,  // 'Y'
0x19, 0x15, 0x13,  // 'Z'
0x1F, 0x11,  // '['
0x3, 0x4, 0x18,  // '\'
0x11, 0x1F,  // ']'
0x2, 0x1, 0x2,  // '^'
0x10, 0x10, 0x10,  // '_'
0x1, 0x2,  // '`'
0x1A, 0x1A, 0x1E,  // 'a'
0x1E, 0x14, 0x1C,  // 'b'
0x1E, 0x12, 0x12,  // 'c'
0x1C, 0x14, 0x1E,  // 'd'
0x1E, 0x16, 0x16,  // 'e'
0x8, 0x1E, 0xA,  // 'f'
0x18, 0x1E, 0x6,  // 'g'
0x1E, 0x4, 0x1C,  // 'h'
0x1A,  // 'i'
0x10, 0x1A,  // 'j'
0x1E, 0x8, 0x14,  // 'k'
0xE, 0x10,  // 'l'
0x1E, 0x4, 0x1E,  // 'm'
0x1E, 0x2, 0x1C,  // 'n'
0x1E, 0x12, 0x1E,  // 'o'
0x1E, 0x6,  // 'p'
0x6, 0x1E,  // 'q'
0x1E, 0x2,  // 'r'
0x14, 0x12, 0xA,  // 's'
0x1E, 0x4,  // 't'
0x1E, 0x10, 0x1E,  // 'u'
0xE, 0x10, 0xE,  // 'v'
0x1E, 0x8, 0x1E,  // 'w'
0x1A, 0x4, 0x1A,  // 'x'
0x18, 0x6, 0x6,  // 'y'
0x1A, 0x16, 0x12,  // 'z'
0x4, 0x1B,  // '{'
0x1B,  // '|'
0x1B, 0x4,  // '}'
0x4, 0x8, 0x4,  // '~'
 // '⌂'
};

const uint8_t font3x5_offsets[][2] = {
 { 0, 0 }, // ' '
 { 0, 1 }, // '!'
 { 1, 2 }, // '"'
 { 3, 3 }, // '#'
 { 6, 3 }, // '$'
 { 9, 3 }, // '%'
 { 12, 3 }, // '&'
 { 15, 1 }, // '''
 { 16, 2 }, // '('
 { 18, 2 }, // ')'
 { 20, 3 }, // '*'
 { 23, 3 }, // '+'
 { 26, 2 }, // ','
 { 28, 3 }, // '-'
 { 31, 1 }, // '.'
 { 32, 3 }, // '/'
 { 35, 3 }, // '0'
 { 38, 2 }, // '1'
 { 40, 3 }, // '2'
 { 43, 3 }, // '3'
 { 46, 3 }, // '4'
 { 49, 3 }, // '5'
 { 52, 3 }, // '6'
 { 55, 3 }, // '7'
 { 58, 3 }, // '8'
 { 61, 3 }, // '9'
 { 64, 1 }, // ':'
 { 65, 2 }, // ';'
 { 67, 3 }, // '<'
 { 70, 3 }, // '='
 { 73, 3 }, // '>'
 { 76, 3 }, // '?'
 { 79, 3 }, // '@'
 { 82, 3 }, // 'A'
 { 85, 3 }, // 'B'
 { 88, 3 }, // 'C'
 { 91, 3 }, // 'D'
 { 94, 3 }, // 'E'
 { 97, 3 }, // 'F'
 { 100, 3 }, // 'G'
 { 103, 3 }, // 'H'
 { 106, 1 }, // 'I'
 { 107, 3 }, // 'J'
 { 110, 3 }, // 'K'
 { 113, 3 }, // 'L'
 { 116, 3 }, // 'M'
 { 119, 3 }, // 'N'
 { 122, 3 }, // 'O'
 { 125, 3 }, // 'P'
 { 128, 3 }, // 'Q'
 { 131, 3 }, // 'R'
 { 134, 3 }, // 'S'
 { 137, 3 }, // 'T'
 { 140, 3 }, // 'U'
 { 143, 3 }, // 'V'
 { 146, 3 }, // 'W'
 { 149, 3 }, // 'X'
 { 152, 3 }, // 'Y'
 { 155, 3 }, // 'Z'
 { 158, 2 }, // '['
 { 160, 3 }, // '\'
 { 163, 2 }, // ']'
 { 165, 3 }, // '^'
 { 168, 3 }, // '_'
 { 171, 2 }, // '`'
 { 173, 3 }, // 'a'
 { 176, 3 }, // 'b'
 { 179, 3 }, // 'c'
 { 182, 3 }, // 'd'
 { 185, 3 }, // 'e'
 { 188, 3 }, // 'f'
 { 191, 3 }, // 'g'
 { 194, 3 }, // 'h'
 { 197, 1 }, // 'i'
 { 198, 2 }, // 'j'
 { 200, 3 }, // 'k'
 { 203, 2 }, // 'l'
 { 205, 3 }, // 'm'
 { 208, 3 }, // 'n'
 { 211, 3 }, // 'o'
 { 214, 2 }, // 'p'
 { 216, 2 }, // 'q'
 { 218, 2 }, // 'r'
 { 220, 3 }, // 's'
 { 223, 2 }, // 't'
 { 225, 3 }, // 'u'
 { 228, 3 }, // 'v'
 { 231, 3 }, // 'w'
 { 234, 3 }, // 'x'
 { 237, 3 }, // 'y'
 { 240, 3 }, // 'z'
 { 243, 2 }, // '{'
 { 245, 1 }, // '|'
 { 246, 2 }, // '}'
 { 248, 3 }, // '~'
 { 251, 0 }, // '⌂'
};