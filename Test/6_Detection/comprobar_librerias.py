import onnxruntime as ort
print(ort.__version__)  # Debería mostrar 1.22.1
print(ort.get_available_providers())  # Debería incluir 'CPUExecutionProvider'