from PIL import Image
import os
import re

input_folder = 'results'
output_gif = 'output.gif'
duration_ms = 100

def extract_number(filename):
    """Extrai o número de arquivos no formato ghi_{float}.png"""
    match = re.search(r"ghi_([0-9]*\.?[0-9]+)\.png", filename)
    return float(match.group(1)) if match else float('-inf')

# Lista e ordena usando o número extraído (MAIOR → MENOR)
image_files = sorted(
    [
        os.path.join(input_folder, f)
        for f in os.listdir(input_folder)
        if f.endswith('.png')
    ],
    key=lambda x: extract_number(os.path.basename(x)),
    reverse=True   # <<< aqui inverte a ordem
)

if not image_files:
    print(f"Nenhum arquivo .png encontrado na pasta '{input_folder}'.")
else:
    images = [Image.open(f) for f in image_files]

    images[0].save(
        output_gif,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=0
    )

    print(f"GIF '{output_gif}' gerado com sucesso contendo {len(image_files)} imagens.")
