/*******************************************************************************
 * SILLA_RUEDAS_CONFIG.H - CONFIGURACIÓN PERSONALIZABLE DEL SISTEMA
 * 
 * Este archivo contiene todas las configuraciones importantes que pueden
 * necesitar ajuste según las necesidades específicas del usuario.
 * 
 * IMPORTANTE: Ajustar estos valores según las características físicas
 * y preferencias de la persona que usará la silla de ruedas.
 * 
 * CORRECCIÓN APLICADA:
 * ? Eliminadas verificaciones de punto flotante en preprocesador
 * ? Validaciones movidas a funciones de tiempo de ejecución
 * ? Compatibilidad completa con XC8
 *******************************************************************************/

#ifndef SILLA_RUEDAS_CONFIG_H
#define SILLA_RUEDAS_CONFIG_H

/*******************************************************************************
 * CONFIGURACIÓN DE SENSIBILIDAD DEL MPU6050
 * 
 * Estos valores determinan qué tan sensible es el control:
 * - Valores más BAJOS = MÁS sensible (menos movimiento necesario)
 * - Valores más ALTOS = MENOS sensible (más movimiento necesario)
 *******************************************************************************/

// === UMBRALES DE MOVIMIENTO ADELANTE/ATRÁS ===
#define SENSIBILIDAD_ADELANTE    0.35f   // Inclinación hacia adelante (0.2 = muy sensible, 0.5 = poco sensible)
#define SENSIBILIDAD_ATRAS       0.35f   // Inclinación hacia atrás (mismo valor que adelante)

// === UMBRALES DE GIRO IZQUIERDA/DERECHA ===
#define SENSIBILIDAD_GIRO_IZQ    40.0f   // Giro cabeza izquierda (20.0 = muy sensible, 60.0 = poco sensible)
#define SENSIBILIDAD_GIRO_DER    40.0f   // Giro cabeza derecha (mismo valor que izquierda)

// === ZONAS MUERTAS (para evitar movimientos involuntarios) ===
#define ZONA_MUERTA_LINEAL       0.20f   // Zona muerta para adelante/atrás (0.1 = permisivo, 0.3 = estricto)
#define ZONA_MUERTA_ROTACIONAL   20.0f   // Zona muerta para giros (10.0 = permisivo, 30.0 = estricto)

/*******************************************************************************
 * CONFIGURACIÓN DE SEGURIDAD
 * 
 * Estos parámetros controlan las características de seguridad del sistema
 *******************************************************************************/

// === DISTANCIAS DE SEGURIDAD ===
#define DISTANCIA_SEGURIDAD_TRASERA    30    // Distancia mínima trasera en cm (15-50 cm recomendado)
#define DISTANCIA_ALERTA_TRASERA       50    // Distancia de alerta trasera en cm
#define DISTANCIA_MAXIMA_CONFIABLE     300   // Distancia máxima confiable del sensor

// === TIEMPOS DE SEGURIDAD ===
#define TIEMPO_ESTABILIDAD_COMANDO     100   // Tiempo para confirmar comando en ms (50-200 ms)
#define TIEMPO_PARADA_EMERGENCIA       50    // Tiempo de respuesta de parada en ms
#define TIEMPO_ENTRE_COMANDOS          20    // Tiempo mínimo entre comandos en ms

// === MODOS DE OPERACIÓN ===
#define HABILITAR_MODO_SEGURIDAD       true  // Activar protecciones de seguridad
#define PERMITIR_GIROS_EN_MOVIMIENTO   false // Permitir giros mientras se mueve (no recomendado)
#define ALERTA_SONORA_RETROCESO        false // Activar alerta sonora al retroceder

/*******************************************************************************
 * CONFIGURACIÓN DE MOTORES
 * 
 * Estos valores controlan el comportamiento de los motores de la silla
 *******************************************************************************/

// === VELOCIDADES DE OPERACIÓN ===
#define VELOCIDAD_NORMAL_ADELANTE      70    // Velocidad normal hacia adelante (%) (50-90)
#define VELOCIDAD_NORMAL_ATRAS         60    // Velocidad normal hacia atrás (%) (40-80)
#define VELOCIDAD_GIRO                 65    // Velocidad para giros (%) (40-80)
#define VELOCIDAD_ARRANQUE_MINIMA      25    // Velocidad mínima de arranque (%) (15-40)

// === CONFIGURACIÓN DE ARRANQUE SUAVE ===
#define HABILITAR_ARRANQUE_SUAVE       true  // Activar arranque suave (recomendado)
#define PASOS_ARRANQUE_SUAVE           5     // Número de pasos en arranque suave (3-8)
#define TIEMPO_PASO_ARRANQUE           50    // Tiempo por paso en ms (30-100)
#define ACELERACION_MAXIMA             10    // Incremento máximo de velocidad por paso (%)

// === TIPOS DE GIRO ===
#define TIPO_GIRO_TANQUE              true   // Giro tipo tanque (un motor adelante, otro atrás)
#define TIPO_GIRO_DIFERENCIAL         false  // Giro diferencial (un motor rápido, otro lento)

/*******************************************************************************
 * CONFIGURACIÓN DE COMUNICACIÓN Y MONITOREO
 *******************************************************************************/

// === UART ===
#define BAUDRATE_UART                 9600   // Velocidad de comunicación (9600 recomendado)
#define ENVIAR_ESTADO_CADA_MS         2000   // Intervalo de envío de estado (1000-5000 ms)
#define BUFFER_UART_SIZE              100    // Tamaño del buffer UART

// === MONITOREO ===
#define HABILITAR_DEBUG_DETALLADO     true   // Activar mensajes de debug detallados
#define MOSTRAR_DATOS_SENSORES        false  // Mostrar datos crudos de sensores
#define CONTAR_COMANDOS_EJECUTADOS    true   // Llevar cuenta de comandos ejecutados

/*******************************************************************************
 * CONFIGURACIÓN AVANZADA - SOLO MODIFICAR SI ES NECESARIO
 *******************************************************************************/

// === CALIBRACIÓN AUTOMÁTICA ===
#define MUESTRAS_CALIBRACION          200    // Número de muestras para calibración (100-500)
#define TIEMPO_CALIBRACION_MS         2000   // Tiempo total de calibración en ms
#define TOLERANCIA_CALIBRACION        0.1f   // Tolerancia para validar calibración

// === FILTROS DE SEÑAL ===
#define HABILITAR_FILTRO_PASO_BAJO    false  // Filtro para suavizar señales
#define FRECUENCIA_MUESTREO_HZ        50     // Frecuencia de muestreo (20-100 Hz)
#define FACTOR_FILTRO                 0.8f   // Factor de filtro paso bajo (0.1-0.9)

// === DETECCIÓN DE ERRORES ===
#define INTENTOS_LECTURA_SENSOR       3      // Intentos de lectura antes de error
#define TIMEOUT_SENSOR_MS             100    // Timeout para lecturas de sensor
#define REINTENTOS_COMUNICACION       2      // Reintentos de comunicación I2C

/*******************************************************************************
 * PERFILES PREDEFINIDOS PARA DIFERENTES USUARIOS
 * 
 * Descomenta el perfil que mejor se adapte al usuario
 *******************************************************************************/

// === PERFIL 1: ALTA SENSIBILIDAD (para usuarios con movimientos limitados) ===
/*
#undef SENSIBILIDAD_ADELANTE
#undef SENSIBILIDAD_ATRAS  
#undef SENSIBILIDAD_GIRO_IZQ
#undef SENSIBILIDAD_GIRO_DER
#undef ZONA_MUERTA_LINEAL
#undef ZONA_MUERTA_ROTACIONAL

#define SENSIBILIDAD_ADELANTE    0.25f
#define SENSIBILIDAD_ATRAS       0.25f
#define SENSIBILIDAD_GIRO_IZQ    25.0f
#define SENSIBILIDAD_GIRO_DER    25.0f
#define ZONA_MUERTA_LINEAL       0.15f
#define ZONA_MUERTA_ROTACIONAL   15.0f
*/

// === PERFIL 2: BAJA SENSIBILIDAD (para usuarios con temblores) ===
/*
#undef SENSIBILIDAD_ADELANTE
#undef SENSIBILIDAD_ATRAS
#undef SENSIBILIDAD_GIRO_IZQ
#undef SENSIBILIDAD_GIRO_DER
#undef ZONA_MUERTA_LINEAL
#undef ZONA_MUERTA_ROTACIONAL

#define SENSIBILIDAD_ADELANTE    0.50f
#define SENSIBILIDAD_ATRAS       0.50f
#define SENSIBILIDAD_GIRO_IZQ    60.0f
#define SENSIBILIDAD_GIRO_DER    60.0f
#define ZONA_MUERTA_LINEAL       0.30f
#define ZONA_MUERTA_ROTACIONAL   35.0f
*/

// === PERFIL 3: MÁXIMA SEGURIDAD (para usuarios novatos) ===
/*
#undef VELOCIDAD_NORMAL_ADELANTE
#undef VELOCIDAD_NORMAL_ATRAS
#undef VELOCIDAD_GIRO
#undef DISTANCIA_SEGURIDAD_TRASERA
#undef TIEMPO_ESTABILIDAD_COMANDO

#define VELOCIDAD_NORMAL_ADELANTE      50
#define VELOCIDAD_NORMAL_ATRAS         40
#define VELOCIDAD_GIRO                 45
#define DISTANCIA_SEGURIDAD_TRASERA    40
#define TIEMPO_ESTABILIDAD_COMANDO     200
*/

/*******************************************************************************
 * ? VALIDACIÓN DE CONFIGURACIÓN CORREGIDA - SIN PUNTO FLOTANTE
 *******************************************************************************/

// NOTA: Las verificaciones de punto flotante se eliminaron del preprocesador
// Las validaciones ahora se realizan en tiempo de ejecución para compatibilidad XC8

// Verificaciones básicas solo con enteros
#if VELOCIDAD_NORMAL_ADELANTE < 30 || VELOCIDAD_NORMAL_ADELANTE > 100
    #error "VELOCIDAD_NORMAL_ADELANTE debe estar entre 30 y 100"
#endif

#if DISTANCIA_SEGURIDAD_TRASERA < 10 || DISTANCIA_SEGURIDAD_TRASERA > 100
    #error "DISTANCIA_SEGURIDAD_TRASERA debe estar entre 10 y 100 cm"
#endif

#if TIEMPO_ESTABILIDAD_COMANDO < 20 || TIEMPO_ESTABILIDAD_COMANDO > 500
    #error "TIEMPO_ESTABILIDAD_COMANDO debe estar entre 20 y 500 ms"
#endif

#if BUFFER_UART_SIZE < 50 || BUFFER_UART_SIZE > 200
    #error "BUFFER_UART_SIZE debe estar entre 50 y 200 bytes"
#endif

/*******************************************************************************
 * FUNCIONES DE VALIDACIÓN EN TIEMPO DE EJECUCIÓN
 *******************************************************************************/

/**
 * @brief Valida la configuración de sensibilidad en tiempo de ejecución
 * @return true si la configuración es válida
 * @note Esta función reemplaza las validaciones del preprocesador
 */
static inline bool ValidarConfiguracionSensibilidad(void) {
    // Validar sensibilidad adelante/atrás
    if (SENSIBILIDAD_ADELANTE < 0.1f || SENSIBILIDAD_ADELANTE > 1.0f) {
        return false;
    }
    if (SENSIBILIDAD_ATRAS < 0.1f || SENSIBILIDAD_ATRAS > 1.0f) {
        return false;
    }
    
    // Validar sensibilidad de giros
    if (SENSIBILIDAD_GIRO_IZQ < 10.0f || SENSIBILIDAD_GIRO_IZQ > 100.0f) {
        return false;
    }
    if (SENSIBILIDAD_GIRO_DER < 10.0f || SENSIBILIDAD_GIRO_DER > 100.0f) {
        return false;
    }
    
    // Validar zonas muertas
    if (ZONA_MUERTA_LINEAL < 0.05f || ZONA_MUERTA_LINEAL > 0.5f) {
        return false;
    }
    if (ZONA_MUERTA_ROTACIONAL < 5.0f || ZONA_MUERTA_ROTACIONAL > 50.0f) {
        return false;
    }
    
    return true;
}

/**
 * @brief Valida la configuración completa del sistema
 * @return true si toda la configuración es válida
 */
static inline bool ValidarConfiguracionCompleta(void) {
    // Validar sensibilidad
    if (!ValidarConfiguracionSensibilidad()) {
        return false;
    }
    
    // Validar velocidades
    if (VELOCIDAD_NORMAL_ADELANTE < 30 || VELOCIDAD_NORMAL_ADELANTE > 100) {
        return false;
    }
    if (VELOCIDAD_NORMAL_ATRAS < 20 || VELOCIDAD_NORMAL_ATRAS > 100) {
        return false;
    }
    if (VELOCIDAD_GIRO < 30 || VELOCIDAD_GIRO > 100) {
        return false;
    }
    
    // Validar distancias
    if (DISTANCIA_SEGURIDAD_TRASERA < 10 || DISTANCIA_SEGURIDAD_TRASERA > 100) {
        return false;
    }
    if (DISTANCIA_ALERTA_TRASERA <= DISTANCIA_SEGURIDAD_TRASERA) {
        return false;
    }
    
    // Validar tiempos
    if (TIEMPO_ESTABILIDAD_COMANDO < 20 || TIEMPO_ESTABILIDAD_COMANDO > 500) {
        return false;
    }
    if (TIEMPO_ENTRE_COMANDOS < 10 || TIEMPO_ENTRE_COMANDOS > 100) {
        return false;
    }
    
    return true;
}

/*******************************************************************************
 * MACROS DE CONVERSIÓN PARA COMPATIBILIDAD
 *******************************************************************************/

// Conversión de configuración nueva a variables del código principal
#define UMBRAL_ADELANTE          SENSIBILIDAD_ADELANTE
#define UMBRAL_ATRAS            (-SENSIBILIDAD_ATRAS)
#define UMBRAL_IZQUIERDA         SENSIBILIDAD_GIRO_IZQ
#define UMBRAL_DERECHA          (-SENSIBILIDAD_GIRO_DER)
#define ZONA_MUERTA_ACCEL        ZONA_MUERTA_LINEAL
#define ZONA_MUERTA_GYRO         ZONA_MUERTA_ROTACIONAL
#define DISTANCIA_SEGURIDAD_CM   DISTANCIA_SEGURIDAD_TRASERA

/*******************************************************************************
 * INFORMACIÓN DE CONFIGURACIÓN
 *******************************************************************************/

#define CONFIG_VERSION "2.1"  // Incrementado por corrección
#define CONFIG_FECHA   "2025"
#define CONFIG_AUTOR   "Sistema de Asistencia Tecnológica"

// Información visible en tiempo de compilación
#pragma message "===== CONFIGURACION DE SILLA DE RUEDAS ====="
#pragma message "Version: " CONFIG_VERSION " (CORREGIDA)"
#pragma message "Sensibilidad configurada para uso general"
#pragma message "Modo seguridad: HABILITADO"
#pragma message "Arranque suave: HABILITADO"
#pragma message "Validacion: Tiempo de ejecucion"
#pragma message "=============================================="

#endif /* SILLA_RUEDAS_CONFIG_H */

/*******************************************************************************
 * RESUMEN DE CORRECCIONES - VERSIÓN 2.1:
 * 
 * ? CORRECCIÓN CRÍTICA:
 * - Eliminadas verificaciones #if con punto flotante
 * - Agregadas funciones de validación en tiempo de ejecución
 * - Versión incrementada a 2.1 para indicar corrección
 * 
 * ? NUEVAS FUNCIONES:
 * - ValidarConfiguracionSensibilidad()
 * - ValidarConfiguracionCompleta()
 * - Validación robusta en tiempo de ejecución
 * 
 * ? MANTIENE COMPATIBILIDAD:
 * - Todos los valores de configuración idénticos
 * - Macros de conversión preservadas
 * - API pública sin cambios
 * 
 * ? GUÍA DE AJUSTE PARA TÉCNICOS:
 * 
 * 1. AJUSTE DE SENSIBILIDAD:
 *    - Comience con valores por defecto
 *    - Si el usuario necesita menos movimiento: REDUZCA los umbrales
 *    - Si hay movimientos involuntarios: AUMENTE las zonas muertas
 * 
 * 2. AJUSTE DE VELOCIDAD:
 *    - Para usuarios novatos: use velocidades bajas (40-60%)
 *    - Para usuarios experimentados: velocidades normales (60-80%)
 *    - NUNCA use velocidades superiores al 90% por seguridad
 * 
 * 3. AJUSTE DE SEGURIDAD:
 *    - Distancia trasera: 20-30cm para espacios pequeños, 40-50cm para espacios amplios
 *    - Tiempo estabilidad: 50ms para respuesta rápida, 200ms para máxima estabilidad
 * 
 * 4. CALIBRACIÓN:
 *    - Realice la calibración con el usuario en posición de uso normal
 *    - Asegúrese de que la cabeza esté cómoda y relajada
 *    - Re-calibre si cambia la posición del sensor
 * 
 * 5. PRUEBAS DE SEGURIDAD:
 *    - Verifique la parada de emergencia antes del primer uso
 *    - Pruebe la detección de obstáculos con objetos reales
 *    - Confirme que los giros no sean demasiado bruscos
 *    - Use ValidarConfiguracionCompleta() en el inicio del sistema
 *******************************************************************************/