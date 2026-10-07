import pandas as pd
import matplotlib.pyplot as plt

# 1. Cargar el archivo CSV
archivo_csv = 'tueste.csv'
df = pd.read_csv(archivo_csv)

# Limpiar espacios en blanco de las columnas
df.columns = df.columns.str.strip()

# 2. Exportar a Excel (.xlsx)
archivo_excel = 'reporte_tueste.xlsx'
df.to_excel(archivo_excel, index=False, sheet_name='Telemetria')
print(f"Archivo Excel generado: {archivo_excel}")

# 3. Graficar
plt.figure(figsize=(10, 6))
plt.plot(df['Tiempo'], df['Temperatura_Real'], label='Temperatura Real (°C)', color='red', linewidth=2)
plt.plot(df['Tiempo'], df['Temperatura_Target'], label='Setpoint Target (°C)', color='blue', linestyle='--', linewidth=1.5)

plt.title('Curva Térmica de Tueste de Café', fontsize=14, fontweight='bold')
plt.xlabel('Tiempo (HH:MM:SS)', fontsize=12)
plt.ylabel('Temperatura (°C)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend(loc='upper left')
plt.xticks(rotation=45, ha='right')
plt.tight_layout()

# 4. Guardar gráfica como imagen
plt.savefig('curva_tueste.png', dpi=300)
print("Gráfica guardada como 'curva_tueste.png'")

# Mostrar en pantalla
plt.show()