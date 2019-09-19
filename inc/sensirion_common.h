/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SENSIRION_COMMON_H
#define SENSIRION_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#define STATUS_OK 0
#define STATUS_FAIL (-1)

#ifndef NULL
#define NULL ((void *)0)
#endif

#if SENSIRION_BIG_ENDIAN
#define be16_to_cpu(s) (s)
#define be32_to_cpu(s) (s)
#define be64_to_cpu(s) (s)
#define SENSIRION_WORDS_TO_BYTES(a, w) ()

#else /* SENSIRION_BIG_ENDIAN */

#define be16_to_cpu(s) (((uint16_t)(s) << 8) | (0xff & ((uint16_t)(s)) >> 8))
#define be32_to_cpu(s) (((u32)be16_to_cpu(s) << 16) | \
                        (0xffff & (be16_to_cpu((s) >> 16))))
#define be64_to_cpu(s) (((u64)be32_to_cpu(s) << 32) | \
                        (0xffffffff & ((u64)be32_to_cpu((s) >> 32))))
/**
 * Convert a word-array to a bytes-array, effectively reverting the
 * host-endianness to big-endian
 * @a:  word array to change (must be (uint16_t *) castable)
 * @w:  number of word-sized elements in the array (SENSIRION_NUM_WORDS(a)).
 */
#define SENSIRION_WORDS_TO_BYTES(a, w) \
    for (uint16_t *__a = (uint16_t *)(a), __e = (w), __w = 0; __w < __e; ++__w) { \
        __a[__w] = be16_to_cpu(__a[__w]); \
    }
#endif /* SENSIRION_BIG_ENDIAN */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

#define CRC8_POLYNOMIAL             0x31
#define CRC8_INIT                   0xFF
#define CRC8_LEN                    1

#define SENSIRION_COMMAND_SIZE      2
#define SENSIRION_WORD_SIZE         2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS  32

uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count);

int8_t sensirion_common_check_crc(uint8_t *data, uint16_t count, uint8_t checksum);

/**
 * sensirion_fill_cmd_send_buf() - create the i2c send buffer for a command and
 *                                 a set of argument words. The output buffer
 *                                 interleaves argument words with their
 *                                 checksums.
 * @buf:        The generated buffer to send over i2c. Then buffer length must
 *              be at least SENSIRION_COMMAND_LEN + num_args *
 *              (SENSIRION_WORD_SIZE + CRC8_LEN).
 * @cmd:        The i2c command to send. It already includes a checksum.
 * @args:       The arguments to the command. Can be NULL if none.
 * @num_args:   The number of word arguments in args.
 *
 * @return      The number of bytes written to buf
 */
#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_COMMON_H */
