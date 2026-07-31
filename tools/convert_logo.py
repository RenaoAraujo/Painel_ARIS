#!/usr/bin/env python3
"""
convert_logo.py — Converte a imagem do logo para binário RGB565 (LVGL 8).

Execute UMA VEZ antes do build:
  cd c:\\Users\\Renan\\Documents\\esp32p4
  python tools/convert_logo.py

Saída: main/logo_aris.bin  (1024 × 600 × 2 bytes = RGB565 little-endian)
Este arquivo é embedado no firmware pelo CMakeLists.txt via EMBED_FILES.

Dependência:  pip install pillow
"""

import os
import struct
import sys
from pathlib import Path

# ── Caminhos ──────────────────────────────────────────────────────────────
SCRIPT_DIR  = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
INPUT       = PROJECT_DIR / "image" / "Logo aris.jpeg"
OUTPUT      = PROJECT_DIR / "main"  / "logo_aris.bin"

# ── Resolução alvo (tela do ESP32-P4) ─────────────────────────────────────
TARGET_W = 1024
TARGET_H = 600

# ── Cor de fundo para as bordas (caso a imagem não preencha tudo) ──────────
# Corresponde ao fundo claro do logo original
BG_R, BG_G, BG_B = 0xEE, 0xEE, 0xF7


def rgb888_to_rgb565_le(r: int, g: int, b: int) -> bytes:
    """Converte um pixel RGB888 para RGB565 little-endian (formato LVGL 8)."""
    rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    return struct.pack("<H", rgb565)


def main() -> None:
    try:
        from PIL import Image
    except ImportError:
        print("ERRO: Pillow não instalado.")
        print("  Execute:  pip install pillow")
        sys.exit(1)

    if not INPUT.exists():
        print(f"ERRO: Imagem não encontrada em {INPUT}")
        sys.exit(1)

    print(f"Abrindo imagem: {INPUT}")
    img = Image.open(INPUT).convert("RGB")
    print(f"  Tamanho original: {img.width} × {img.height}")

    # ── Redimensiona mantendo proporção, sem ultrapassar 1024×600 ──────────
    img.thumbnail((TARGET_W, TARGET_H), Image.LANCZOS)
    w, h = img.size
    print(f"  Após resize:      {w} × {h}")

    # ── Centraliza em canvas 1024×600 com cor de fundo ──────────────────
    canvas = Image.new("RGB", (TARGET_W, TARGET_H), (BG_R, BG_G, BG_B))
    x_off  = (TARGET_W - w) // 2
    y_off  = (TARGET_H - h) // 2
    canvas.paste(img, (x_off, y_off))
    print(f"  Centralizado em canvas {TARGET_W} × {TARGET_H}")

    # ── Gera binário RGB565 ──────────────────────────────────────────────
    print("Convertendo para RGB565 little-endian…")
    buf = bytearray(TARGET_W * TARGET_H * 2)
    idx = 0
    pixels = canvas.load()
    for y in range(TARGET_H):
        for x in range(TARGET_W):
            r, g, b = pixels[x, y]
            word = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            buf[idx]     = word & 0xFF
            buf[idx + 1] = (word >> 8) & 0xFF
            idx += 2

    OUTPUT.write_bytes(buf)
    size_kb = len(buf) / 1024
    print(f"Arquivo gerado: {OUTPUT}  ({size_kb:.0f} KB)")
    print()
    print("Agora compile normalmente: idf.py build")


if __name__ == "__main__":
    main()
