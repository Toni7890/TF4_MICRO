/*******************************************************************************
 * SILLA_RUEDAS_CONFIG.H - CONFIGURACI�N PERSONALIZABLE DEL SISTEMA
 * 
 * Este archivo contiene todas las configuraciones importantes que pueden
 * necesitar ajuste seg�n las necesidades espec�ficas del usuario.
 * 
 * IMPORTANTE: Ajustar estos valores seg�n las caracter�sticas f�sicas
 * y preferencias de la persona que usar� la silla de ruedas.
 * 
 * CORRECCI�N APLICADA:
 * ? Eliminadas verificaciones de punto flotante en preprocesador
 * ? Validaciones movidas a funciones de tiempo de ejecuci�n
 * ? Compatibilidad completa con XC8
 *******************************************************************************/

#ifndef SILLA_RUEDAS_CONFIG_H
#define SILLA_RUEDAS_CONFIG_H

/*******************************************************************************
 * CONFIGURACI�N DE SENSIBILIDAD DEL MPU6050
 * 
 * Estos valores determinan qu� tan sensible es el control:
 * - Valores m�s BAJOS = M�S sensible (menos movimiento necesario)
 * - Valores m�s ALTOS = MENOS sensible (m�s movimiento necesario)
 *******************************************************************************/

// === UMBRALES DE MOVIMIENTO ADELANTE/ATR�S ===
#define SENSIBILIDAD_ADELANTE    0.35f   // Inclinaci�n hacia adelante (0.2 = muy sensible, 0.5 = poco sensible)
#define SENSIBILIDAD_ATRAS       0.35f   // Inclinaci�n hacia atr�s (mismo valor que adelante)

// === UMBRALES DE GIRO IZQUIERDA/DERECHA ===
#define SENSIBILIDAD_GIRO_IZQ    40.0f   // Giro cabeza izquierda (20.0 = muy sensible, 60.0 = poco sensible)
#define SENSIBILIDAD_GIRO_DER    40.0f   // Giro cabeza derecha (mismo valor que izquierda)

// === ZONAS MUERTAS (para evitar movimientos involuntarios) ===
#define ZONA_MUERTA_LINEAL       0.20f   // Zona muerta para adelante/atr�s (0.1 = permisivo, 0.3 = estricto)
#define ZONA_MUERTA_ROTACIONAL   20.0f   // Zona muerta para giros (10.0 = permisivo, 30.0 = estricto)

/*******************************************************************************
 * CONFIGURACI�N DE SEGURIDAD
 * 
 * Estos par�metros controlan las caracter�sticas de seguridad del sistema
 *******************************************************************************/

// === DISTANCIAS DE SEGURIDAD ===
#define DISTANCIA_SEGURIDAD_TRASERA    30    // Distancia m�nima trasera en cm (15-50 cm recomendado)
#define DISTANCIA_ALERTA_TRASERA       50    // Distancia de alerta trasera en cm
#define DISTANCIA_MAXIMA_CONFIABLE     300   // Distancia m�xima confiable del sensor

// === TIEMPOS DE SEGURIDAD ===
#define TIEMPO_ESTABILIDAD_COMANDO     100   // Tiempo para confirmar comando en ms (50-200 ms)
#define TIEMPO_PARADA_EMERGENCIA       50    // Tiempo de respuesta de parada en ms
#define TIEMPO_ENTRE_COMANDOS          20    // Tiempo m�nimo entre comandos en ms

// === MODOS DE OPERACI�N ===
#define HABILITAR_MODO_SEGURIDAD       true  // Activar protecciones de seguridad
#define PERMITIR_GIROS_EN_MOVIMIENTO   false // Permitir giros mientras se mueve (no recomendado)
#define ALERTA_SONORA_RETROCESO        false // Activar alerta sonora al retroceder

/*******************************************************************************
 * CONFIGURACI�N DE MOTORES
 * 
 * Estos valores controlan el comportamiento de los motores de la silla
 *******************************************************************************/

// === VELOCIDADES DE OPERACI�N ===
#define VELOCIDAD_NORMAL_ADELANTE      70    // Velocidad normal hacia adelante (%) (50-90)
#define VELOCIDAD_NORMAL_ATRAS         60    // Velocidad normal hacia atr�s (%) (40-80)
#define VELOCIDAD_GIRO                 65    // Velocidad para giros (%) (40-80)
#define VELOCIDAD_ARRANQUE_MINIMA      25    // Velocidad m�nima de arranque (%) (15-40)

// === CONFIGURACI�N DE ARRANQUE SUAVE ===
#define HABILITAR_ARRANQUE_SUAVE       true  // Activar arranque suave (recomendado)
#define PASOS_ARRANQUE_SUAVE           5     // N�mero de pasos en arranque suave (3-8)
#define TIEMPO_PASO_ARRANQUE           50    // Tiempo por paso en ms (30-100)
#define ACELERACION_MAXIMA             10    // Incremento m�ximo de velocidad por paso (%)

// === TIPOS DE GIRO ===
#define TIPO_GIRO_TANQUE              true   // Giro tipo tanque (un motor adelante, otro atr�s)
#define TIPO_GIRO_DIFERENCIAL         false  // Giro diferencial (un motor r�pido, otro lento)

/*******************************************************************************
 * CONFIGURACI�N DE COMUNICACI�N Y MONITOREO
 *******************************************************************************/

// === UART ===
#define BAUDRATE_UART                 9600   // Velocidad de comunicaci�n (9600 recomendado)
#define ENVIAR_ESTADO_CADA_MS         2000   // Intervalo de env�o de estado (1000-5000 ms)
#define BUFFER_UART_SIZE              100    // Tama�o del buffer UART

// === MONITOREO ===
#define HABILITAR_DEBUG_DETALLADO     true   // Activar mensajes de debug detallados
#define MOSTRAR_DATOS_SENSORES        false  // Mostrar datos crudos de sensores
#define CONTAR_COMANDOS_EJECUTADOS    true   // Llevar cuenta de comandos ejecutados

/*******************************************************************************
 * CONFIGURACI�N AVANZADA - SOLO MODIFICAR SI ES NECESARIO
 *******************************************************************************/

// === CALIBRACI�N AUTOM�TICA ===
#define MUESTRAS_CALIBRACION          200    // N�mero de muestras para calibraci�n (100-500)
#define TIEMPO_CALIBRACION_MS         2000   // Tiempo total de calibraci�n en ms
#define TOLERANCIA_CALIBRACION        0.1f   // Tolerancia para validar calibraci�n

// === FILTROS DE SE�AL ===
#define HABILITAR_FILTRO_PASO_BAJO    false  // Filtro para suavizar se�ales
#define FRECUENCIA_MUESTREO_HZ        50     // Frecuencia de muestreo (20-100 Hz)
#define FACTOR_FILTRO                 0.8f   // Factor de filtro paso bajo (0.1-0.9)

// === DETECCI�N DE ERRORES ===
#define INTENTOS_LECTURA_SENSOR       3      // Intentos de lectura antes de error
#define TIMEOUT_SENSOR_MS             100    // Timeout para lecturas de sensor
#define REINTENTOS_COMUNICACION       2      // Reintentos de comunicaci�n I2C

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

// === PERFIL 3: M�XIMA SEGURIDAD (para usuarios novatos) ===
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
 * ? VALIDACI�N DE CONFIGURACI�N CORREGIDA - SIN PUNTO FLOTANTE
 *******************************************************************************/

// NOTA: Las verificaciones de punto flotante se eliminaron del preprocesador
// Las validaciones ahora se realizan en tiempo de ejecuci�n para compatibilidad XC8

// Verificaciones b�sicas solo con enteros
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
 * FUNCIONES DE VALIDACI�N EN TIEMPO DE EJECUCI�N
 *******************************************************************************/

/**
 * @brief Valida la configuraci�n de sensibilidad en tiempo de ejecuci�n
 * @return true si la configuraci�n es v�lida
 * @note Esta funci�n reemplaza las validaciones del preprocesador
 */
static inline bool ValidarConfiguracionSensibilidad(void) {
    // Validar sensibilidad adelante/atr�s
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
 * @brief Valida la configuraci�n completa del sistema
 * @return true si toda la configuraci�n es v�lida
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
 * MACROS DE CONVERSI�N PARA COMPATIBILIDAD
 *******************************************************************************/

// Conversi�n de configuraci�n nueva a variables del c�digo principal
#define UMBRAL_ADELANTE          SENSIBILIDAD_ADELANTE
#define UMBRAL_ATRAS            (-SENSIBILIDAD_ATRAS)
#define UMBRAL_IZQUIERDA         SENSIBILIDAD_GIRO_IZQ
#define UMBRAL_DERECHA          (-SENSIBILIDAD_GIRO_DER)
#define ZONA_MUERTA_ACCEL        ZONA_MUERTA_LINEAL
#define ZONA_MUERTA_GYRO         ZONA_MUERTA_ROTACIONAL
#define DISTANCIA_SEGURIDAD_CM   DISTANCIA_SEGURIDAD_TRASERA

/*******************************************************************************
 * INFORMACI�N DE CONFIGURACI�N
 *******************************************************************************/

#define CONFIG_VERSION "2.1"  // Incrementado por correcci�n
#define CONFIG_FECHA   "2025"
#define CONFIG_AUTOR   "Sistema de Asistencia Tecnol�gica"

// Informaci�n visible en tiempo de compilaci�n
#pragma message "===== CONFIGURACION DE SILLA DE RUEDAS ====="
#pragma message "Version: " CONFIG_VERSION " (CORREGIDA)"
#pragma message "Sensibilidad configurada para uso general"
#pragma message "Modo seguridad: HABILITADO"
#pragma message "Arranque suave: HABILITADO"
#pragma message "Validacion: Tiempo de ejecucion"
#pragma message "=============================================="

#endif /* SILLA_RUEDAS_CONFIG_H */

/*******************************************************************************
 * RESUMEN DE CORRECCIONES - VERSI�N 2.1:
 * 
 * ? CORRECCI�N CR�TICA:
 * - Eliminadas verificaciones #if con punto flotante
 * - Agregadas funciones de validaci�n en tiempo de ejecuci�n
 * - Versi�n incrementada a 2.1 para indicar correcci�n
 * 
 * ? NUEVAS FUNCIONES:
 * - ValidarConfiguracionSensibilidad()
 * - ValidarConfiguracionCompleta()
 * - Validaci�n robusta en tiempo de ejecuci�n
 * 
 * ? MANTIENE COMPATIBILIDAD:
 * - Todos los valores de configuraci�n id�nticos
 * - Macros de conversi�n preservadas
 * - API p�blica sin cambios
 * 
 * ? GU�A DE AJUSTE PARA T�CNICOS:
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
 *    - Distancia trasera: 20-30cm para espacios peque�os, 40-50cm para espacios amplios
 *    - Tiempo estabilidad: 50ms para respuesta r�pida, 200ms para m�xima estabilidad
 * 
 * 4. CALIBRACI�N:
 *    - Realice la calibraci�n con el usuario en posici�n de uso normal
 *    - Aseg�rese de que la cabeza est� c�moda y relajada
 *    - Re-calibre si cambia la posici�n del sensor
 * 
 * 5. PRUEBAS DE SEGURIDAD:
 *    - Verifique la parada de emergencia antes del primer uso
 *    - Pruebe la detecci�n de obst�culos con objetos reales
 *    - Confirme que los giros no sean demasiado bruscos
 *    - Use ValidarConfiguracionCompleta() en el inicio del sistema
 *******************************************************************************/