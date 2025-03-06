/****************************************************************************
 * Incluimos las librerías necesarias para trabajar con micro-ROS en Arduino,*
 * además de la librería estándar de Arduino.                               *
 ****************************************************************************/
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Arduino.h>

/****************************************************************************
 * Definición de los pines usados para el puente H y el PWM.               *
 * Ajustar estos valores según el hardware (Hackerboard o ESP32).          *
 ****************************************************************************/
#define MOTOR_PWM_PIN  4    // Pin físico que se conecta a la entrada PWM del driver de motor (GPIO 04)
#define MOTOR_IN1_PIN  18   // Pin físico que se conecta a la entrada IN1 del driver (GPIO 18)
#define MOTOR_IN2_PIN  15   // Pin físico que se conecta a la entrada IN2 del driver (GPIO 15)

// Pin que usaremos para parpadear en caso de error (LED integrado o externo).
#define ERROR_LED_PIN  2    

/****************************************************************************
 * Configuración del PWM:                                                  *
 * - Frecuencia en Hz (aquí 980 Hz).                                       *
 * - Resolución en bits (aquí 8 bits -> valores de 0 a 255).               *
 * - Canal de PWM que vamos a usar (0, pero puede haber hasta 16).         *
 ****************************************************************************/
#define PWM_FREQ       980  
#define PWM_RESOLUTION 8    
#define PWM_CHANNEL    0    

/****************************************************************************
 * Definimos el rango del mensaje que vamos a recibir en /cmd_pwm.         *
 * Aquí, -1.0f indica giro inverso al 100% y 1.0f giro directo al 100%.     *
 * El valor 0.0f detiene el motor.                                         *
 ****************************************************************************/
#define MIN_CMD -1.0f
#define MAX_CMD  1.0f

/****************************************************************************
 * Declaración de objetos y variables propias de micro-ROS:                *
 * - node              : Representa el nodo en ROS 2.                       *
 * - executor          : Maneja la ejecución de callbacks (suscripciones).  *
 * - support           : Estructura de soporte para inicializar micro-ROS.  *
 * - allocator         : Administrador de memoria.                          *
 * - subscriber        : Estructura de suscripción al tópico.               *
 * - msg               : Mensaje (Float32) que recibimos.                   *
 ****************************************************************************/
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg; 

/****************************************************************************
 * Macros para simplificar la verificación de errores en llamadas rcl.     *
 * - RCCHECK(fn)   : Si falla la función fn, llama a error_loop().          *
 * - RCSOFTCHECK(fn): Hace la llamada fn, pero ignora el error si ocurre.   *
 ****************************************************************************/
#define RCCHECK(fn)  { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

/****************************************************************************
 * Función de error: se llama cuando alguna inicialización o llamada falla. *
 * Parpadea el LED definido en ERROR_LED_PIN indefinidamente.               *
 ****************************************************************************/
void error_loop() {
  pinMode(ERROR_LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}

/****************************************************************************
 * Callback que se llama al recibir un nuevo mensaje en el tópico /cmd_pwm. *
 *                                                                          *
 * El mensaje es de tipo Float32 y se asume que viene en el rango [-1,1].   *
 * - Si el valor es positivo, el motor gira hacia adelante.                 *
 * - Si el valor es negativo, el motor gira hacia atrás.                    *
 * - Si el valor es cero, se detiene el motor.                              *
 *                                                                          *
 * El valor absoluto del mensaje define el nivel de PWM (0% a 100%).        *
 ****************************************************************************/
void subscription_callback(const void *msg_in)
{
  // Convertimos el puntero genérico msg_in al tipo std_msgs__msg__Float32
  const std_msgs__msg__Float32 *rcv_msg = (const std_msgs__msg__Float32 *)msg_in;
  float cmd_value = rcv_msg->data;  // Guardamos el valor recibido

  // Aseguramos que cmd_value no exceda los límites [-1,1]
  if (cmd_value > MAX_CMD) cmd_value = MAX_CMD;
  if (cmd_value < MIN_CMD) cmd_value = MIN_CMD;

  // Lógica para decidir la dirección según el signo de cmd_value
  if (cmd_value > 0.0f) {
    // Giro hacia adelante: IN1 = HIGH, IN2 = LOW
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else if (cmd_value < 0.0f) {
    // Giro hacia atrás: IN1 = LOW, IN2 = HIGH
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  } else {
    // cmd_value == 0 -> Detener motor: IN1 = LOW, IN2 = LOW
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }

  // Obtenemos el valor absoluto de cmd_value para calcular el duty cycle
  float duty_norm = fabs(cmd_value);          // duty_norm va de 0 a 1
  uint8_t duty = (uint8_t)(duty_norm * 255.0f); // Convertimos [0,1] a [0,255]

  // Aplicamos el duty cycle al canal PWM (PWM_CHANNEL)
  ledcWrite(PWM_CHANNEL, duty);
}

/****************************************************************************
 * setup(): Se ejecuta una vez al inicio.                                   *
 *  1. Configura la comunicación de micro-ROS (ej. vía Serial).             *
 *  2. Configura pines de motor como salida y PWM.                          *
 *  3. Inicializa Micro-ROS (allocator, support, node, etc.).               *
 *  4. Crea y configura la suscripción al tópico "/cmd_pwm".               *
 *  5. Inicia el executor para procesar callbacks.                          *
 ****************************************************************************/
void setup() {
  // Inicializamos la comunicación con el agente micro-ROS
  set_microros_transports();

  // Configuramos los pines del motor como salidas digitales
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // Configuramos el canal PWM en la frecuencia y resolución deseadas
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  // Asignamos el pin físico (MOTOR_PWM_PIN) al canal de PWM
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);

  // Esperamos 2 segundos para asegurar que el agente se conecte correctamente
  delay(2000);

  // Inicializamos el allocator por defecto
  allocator = rcl_get_default_allocator();

  // Inicializamos la estructura de soporte de micro-ROS
  // (maneja el contexto y la comunicación con el agente)
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Creamos el nodo de micro-ROS con nombre "motor"
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // Creamos la suscripción al tópico "/cmd_pwm" de tipo Float32
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm"
  ));

  // Inicializamos el executor con capacidad para manejar 1 'handle' (la suscripción)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Agregamos la suscripción al executor y le indicamos la función callback
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA
  ));
}

/****************************************************************************
 * loop(): Se ejecuta en bucle infinito.                                    *
 *  - Aquí llamamos a rclc_executor_spin_some() para procesar callbacks     *
 *    pendientes (cuando llegan mensajes nuevos).                           *
 *  - Usamos un delay(50) para no saturar la CPU con spin en cada ciclo.    *
 ****************************************************************************/
void loop() {
  // Procesamos las llamadas pendientes del executor (mensajes, etc.)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Pausa breve para evitar un bucle muy rápido
  delay(50);
}
