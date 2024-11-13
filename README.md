El **Filtro de Kalman** es un algoritmo recursivo utilizado para la estimación de variables de estado en sistemas dinámicos lineales y con ruido gaussiano. Su objetivo es predecir y ajustar el estado de un sistema basándose en una serie de observaciones ruidosas e imprecisas. El filtro es ampliamente usado en campos como la navegación, control de vehículos, robótica, y visión por computadora.

### Componentes Clave del Filtro de Kalman

1. **Estado del sistema (\(x\))**: Representa las variables internas que describen el sistema (por ejemplo, posición y velocidad de un vehículo).
   
2. **Modelo de transición (\(A\))**: Describe cómo el estado cambia con el tiempo de acuerdo a la dinámica del sistema.

3. **Modelo de control (\(B\))**: Define cómo las entradas de control afectan el estado.

4. **Modelo de observación (\(H\))**: Relaciona el estado verdadero del sistema con las observaciones disponibles.

5. **Covarianza de proceso (\(Q\))** y **covarianza de observación (\(R\))**: Representan las incertidumbres en el modelo de transición y en las observaciones, respectivamente.

### Proceso del Algoritmo

El Filtro de Kalman se divide en dos pasos principales:

1. **Predicción**:
   - **Predicción del estado**: Se estima el estado actual del sistema usando el estado anterior y el modelo de transición.
   - **Predicción de la covarianza**: Se actualiza la covarianza de la estimación para reflejar la incertidumbre esperada en el nuevo estado.

  ![imagen](https://github.com/user-attachments/assets/291f529b-2458-4490-844c-d1e1c3b77db6)


2. **Actualización**:
   - **Cálculo de la ganancia de Kalman (\(K\))**: Determina cuánto peso se debe dar a la nueva observación en comparación con la predicción.
   - **Actualización del estado**: Ajusta la estimación del estado utilizando la observación.
   - **Actualización de la covarianza**: Ajusta la incertidumbre de la estimación del estado.

![imagen](https://github.com/user-attachments/assets/b5242efb-ab31-4d72-91a3-8eb53af0ad18)


### Ventajas del Filtro de Kalman

- **Computacionalmente eficiente**: Utiliza cálculos matriciales simples y es apto para sistemas en tiempo real.
- **Robusto a ruido**: Maneja bien el ruido en el sistema y en las observaciones.
- **Recursivo**: No necesita almacenar todas las observaciones pasadas, solo el estado actual estimado y la covarianza.

En resumen, el Filtro de Kalman proporciona una estimación óptima del estado de un sistema lineal con ruido gaussiano, combinando predicciones basadas en el modelo del sistema con observaciones ruidosas de manera efectiva.
