"""Convert a raw IDEC RGB32 output dump (macroblock order) into a PNG.

usage: ipu_dump_to_png.py ipu_picture.bin <width_in_pixels> out.png
"""
import struct
import sys
import zlib


def main():
    data = open(sys.argv[1], 'rb').read()
    width = int(sys.argv[2])
    mbw = width // 16
    mbs = len(data) // 1024
    mbh = (mbs + mbw - 1) // mbw
    height = mbh * 16
    pixels = bytearray(width * height * 4)
    for mb in range(mbs):
        mx, my = mb % mbw, mb // mbw
        block = data[mb * 1024:(mb + 1) * 1024]
        for row in range(16):
            dst = ((my * 16 + row) * width + mx * 16) * 4
            pixels[dst:dst + 64] = block[row * 64:(row + 1) * 64]
    for i in range(3, len(pixels), 4):
        pixels[i] = 255
    raw = b''.join(b'\x00' + bytes(pixels[y * width * 4:(y + 1) * width * 4]) for y in range(height))

    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))

    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b'')
    open(sys.argv[3], 'wb').write(png)


if __name__ == '__main__':
    main()
