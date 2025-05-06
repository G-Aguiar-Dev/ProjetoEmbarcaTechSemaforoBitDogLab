//Inclusão de bibliotecas
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "matriz_LED.pio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>

// Definições de constantes do display
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Definições de macros
#define BUTTON_A 5
#define BUZZER_A 21
#define BUZZER_B 10
#define LED_MATRIX 7
#define LED_PIN_GREEN 11
#define LED_PIN_RED 13

volatile bool diurno = true; // Variável para controlar o modo diurno ou noturno

void vSetStateTask() // Task para alteração de estado de diurno para noturno
{
    gpio_init(BUTTON_A);                // Inicializa o botão A
    gpio_set_dir(BUTTON_A, GPIO_IN);    // Configura o botão A como entrada
    gpio_pull_up(BUTTON_A);             // Ativa o pull-up interno do botão A

    while (true)
    {
        if (!gpio_get(BUTTON_A)) // Verifica se o botão A está pressionado
        {
            vTaskDelay(pdMS_TO_TICKS(200));     // Aguarda 200ms para debounce
            if (!gpio_get(BUTTON_A))            // Verifica novamente se o botão A está pressionado
            {
                diurno = !diurno;               // Alterna o modo diurno/noturno
                vTaskDelay(pdMS_TO_TICKS(200)); // Aguarda 200ms para debounce
            }  
       }
    }
}

volatile bool led_state_green = false;  // Variável para controlar o estado do LED verde
volatile bool led_state_red = false;    // Variável para controlar o estado do LED vermelho

void vLEDS_SemaforoTask() // Task para os LEDs do semáforo
{
    gpio_init(LED_PIN_GREEN);               // Inicializa o LED verde
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);  // Configura o LED verde como saída
    gpio_init(LED_PIN_RED);                 // Inicializa o LED vermelho
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);    // Configura o LED vermelho como saída
    while (true)
    {
        if (diurno){        // Modo Diurno
            for (int verde = 0; verde < 20; verde++){       // Loop para o sinal verde (20 repetições de 1s)
                if (diurno){
                    gpio_put(LED_PIN_RED, false);       // Apaga o LED vermelho
                    led_state_red = false;              // Atualiza o estado do LED vermelho
                    gpio_put(LED_PIN_GREEN, true);      // Acende o LED verde (Sinal Verde)
                    led_state_green = true;             // Atualiza o estado do LED verde
                    vTaskDelay(pdMS_TO_TICKS(1000));    // Aguarda 1s
                } else {
                    continue;                           // Sai do loop se o modo for noturno
                    }
            }
            for (int amarelo = 0; amarelo < 10; amarelo++){ // Loop para o sinal amarelo (10 repetições de 400ms) 
                if (diurno){
                    gpio_put(LED_PIN_RED, true);        // Acende o LED vermelho e mantém o verde (Sinal Amarelo)
                    led_state_red = true;               // Atualiza o estado do LED vermelho
                    vTaskDelay(pdMS_TO_TICKS(400));     // Aguarda 400ms
                } else {
                    continue; // Sai do loop se o modo for noturno
                    }
            }
            for (int vermelho = 0; vermelho < 8; vermelho++){ // Loop para o sinal vermelho (8 repetições de 2s 
                if (diurno){
                    gpio_put(LED_PIN_GREEN, false);     // Apaga o LED verde
                    led_state_green = false;            // Atualiza o estado do LED verde
                    gpio_put(LED_PIN_RED, true);        // Acende o LED vermelho (Sinal Vermelho)
                    led_state_red = true;               // Atualiza o estado do LED vermelho
                    vTaskDelay(pdMS_TO_TICKS(2000));    // Aguarda 2s
                } else {
                    continue;                           // Sai do loop se o modo for noturno
                }
            }
        } else {            // Modo Noturno
            gpio_put(LED_PIN_GREEN, true);              // Acende o LED verde
            led_state_green = false;                    // Atualiza o estado do LED verde
            gpio_put(LED_PIN_RED, true);                // Acende o LED vermelho
            led_state_red = false;                      // Atualiza o estado do LED vermelho
            vTaskDelay(pdMS_TO_TICKS(200));             // Aguarda 200ms
            gpio_put(LED_PIN_GREEN, false);             // Apaga o LED verde
            gpio_put(LED_PIN_RED, false);               // Apaga o LED vermelho
            vTaskDelay(pdMS_TO_TICKS(1800));            // Aguarda 1800ms
        }
    }
}

// Matriz de LEDs

#define NUM_PIXELS 25 // Número de LEDs na matriz
double led_buffer[25][3] = {0}; // Buffer para armazenar o estado dos LEDs

uint matrix_rgb(float r, float g, float b) // Função auxiliar para converter RGB em um valor de 32 bits
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

void desenho_pio(double desenho[25][3], uint32_t valor_led, PIO pio, uint sm) // Função auxiliar para desenhar na matriz
{

  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    valor_led = matrix_rgb(desenho[i][0], desenho[i][1], desenho[i][2]);
    pio_sm_put_blocking(pio, sm, valor_led);
  };
}


double apagar_leds[25][3] =      // Apagar LEDs da matriz
 {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};


void atualizar_livre() {    // Atualiza os LEDs para o estado livre (verde)
    // Define os LEDs que devem estar acesos (verde) e apaga os outros
  for (int i = 0; i < NUM_PIXELS; i++) {
    if (i == 1 || i == 3 || i == 5 || i == 7 || i == 9 || i == 11 || i == 12 || i == 13 || i == 16 || i == 17|| i == 18 || i == 22){
        double verde[3] = {0.0, 1.0, 0.0};  // Verde
        led_buffer[i][0] = verde[0];
        led_buffer[i][1] = verde[1];
        led_buffer[i][2] = verde[2];
        }
        else {
            led_buffer[i][0] = 0.0; // Apaga o LED
            led_buffer[i][1] = 0.0; // Apaga o LED
            led_buffer[i][2] = 0.0; // Apaga o LED
        }
    }
}

void atualizar_pare() {     // Atualiza os LEDs para o estado de pare (vermelho)
    // Define os LEDs que devem estar acesos (vermelho) e apaga os outros
    for (int i = 0; i < NUM_PIXELS; i++) {
      if (i == 1 || i == 3 || i == 6 || i == 7 || i == 8 || i == 10 || i == 11 || i == 12 ||
          i == 13 || i == 14 || i == 15 || i == 16 || i == 17|| i == 18 || i == 19 || i == 22){
          double vermelho[3] = {1.0, 0.0, 0.0}; // Vermelho
          led_buffer[i][0] = vermelho[0];
          led_buffer[i][1] = vermelho[1];
          led_buffer[i][2] = vermelho[2];
        }
        else {
            led_buffer[i][0] = 0.0; // Apaga o LED
            led_buffer[i][1] = 0.0; // Apaga o LED
            led_buffer[i][2] = 0.0; // Apaga o LED
        }
    }
}

void vMatrizTask() // Task para controlar a matriz de LEDs
{
    PIO pio = pio0;
    bool frequenciaClock;
    uint16_t i;
    uint valor_led;
    frequenciaClock = set_sys_clock_khz(128000, false);     // frequência de clock de 128MHz
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, LED_MATRIX);   // Inicializa o PIO para a matriz de LEDs

    desenho_pio(apagar_leds, valor_led, pio, sm); // Apaga os LEDs atuais da matriz

    while(true)
    {
        if (diurno)     // Modo Diurno
        {
            if (led_state_green && !led_state_red) // Sinal Verde
            {
                atualizar_pare();                               // Atualiza os LEDs para vermelho
                desenho_pio(led_buffer, valor_led, pio, sm);    // Desenha os LEDs na matriz
                vTaskDelay(pdMS_TO_TICKS(300));                 // Aguarda 300ms
            }
            else if (led_state_green && led_state_red) // Sinal Amarelo
            {
                desenho_pio(apagar_leds, valor_led, pio, sm);   // Apaga os LEDs atuais da matriz
                vTaskDelay(pdMS_TO_TICKS(200));                 // Aguarda 200ms
                atualizar_pare();                               // Atualiza os LEDs para vermelho
                desenho_pio(led_buffer, valor_led, pio, sm);    // Desenha os LEDs na matriz
                vTaskDelay(pdMS_TO_TICKS(200));                 // Aguarda 200ms
            }
            else if (!led_state_green && led_state_red) // Sinal Vermelho
            atualizar_livre();                                  // Atualiza os LEDs para verde
            desenho_pio(led_buffer, valor_led, pio, sm);        // Desenha os LEDs na matriz
            vTaskDelay(pdMS_TO_TICKS(300));                     // Aguarda 300ms
        }
        else            // Modo Noturno
        {
            desenho_pio(apagar_leds, valor_led, pio, sm);       // Desenha os LEDs na matriz
            vTaskDelay(pdMS_TO_TICKS(300));                     // Aguarda 300ms
        }
    }
}

void pwm_setup(uint8_t GPIO) {  // Função auxiliar para configurar o PWM
    gpio_init(GPIO);                                // Inicializa o pino GPIO
    gpio_set_dir(GPIO, GPIO_OUT);                   // Configura o pino como saída
    gpio_set_function(GPIO, GPIO_FUNC_PWM);         // Define função PWM para o pino
    pwm_set_enabled(GPIO, true);                    // Habilita o PWM
    uint slice_num = pwm_gpio_to_slice_num(GPIO);   // Obtém o número do slice
    pwm_config config = pwm_get_default_config();   // Configuração padrão
    pwm_config_set_wrap(&config, 1);                // Wrap em 1
    pwm_config_set_clkdiv(&config, 1.0f);           // Divisor de clock em 1.0
    pwm_init(slice_num, &config, true);             // Inicializa PWM
    pwm_set_gpio_level(GPIO, 0);                    // Inicializa o nível do PWM em 0
}

void vBuzzerTask()
{
    pwm_setup(BUZZER_A); // Configura o pino do buzzer A para PWM
    pwm_setup(BUZZER_B); // Configura o pino do buzzer B para PWM
    while (true)
    {
        if (!diurno){ // Modo Noturno (Intervalos de 2s)  
        {
            pwm_set_gpio_level(BUZZER_A, 1);
            pwm_set_gpio_level(BUZZER_B, 1); // Liga o buzzer A e B
            vTaskDelay(pdMS_TO_TICKS(200));  // Aguarda 200ms
            pwm_set_gpio_level(BUZZER_A, 0);
            pwm_set_gpio_level(BUZZER_B, 0); // Desliga o buzzer A e B
            vTaskDelay(pdMS_TO_TICKS(1800)); // Aguarda 1800ms
            }
        }
        else // Modo Diurno
        {
        if (led_state_green && !led_state_red){
            for (int verde = 0; verde < 20; verde++){       // Loop para o sinal verde (20 repetições de 1s)
                if (diurno){                
                    pwm_set_gpio_level(BUZZER_A, 1);
                    pwm_set_gpio_level(BUZZER_B, 1);    // Liga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(200));     // Aguarda 200ms
                    pwm_set_gpio_level(BUZZER_A, 0);    
                    pwm_set_gpio_level(BUZZER_B, 0);    // Desliga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(800));     // Aguarda 800ms
                } else {
                    break;                              // Sai do loop se o modo for noturno
                }
            }
        } else if (led_state_green && led_state_red){
            for (int amarelo = 0; amarelo < 10; amarelo++){   // Loop para o sinal amarelo (10 repetições de 400ms)
                if (diurno){
                    pwm_set_gpio_level(BUZZER_A, 1);
                    pwm_set_gpio_level(BUZZER_B, 1);    // Liga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(200));     // Aguarda 200ms
                    pwm_set_gpio_level(BUZZER_A, 0);    
                    pwm_set_gpio_level(BUZZER_B, 0);    // Desliga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(200));     // Aguarda 200ms
                } else {
                    break;                              // Sai do loop se o modo for noturno
                }
            }
        } else if (!led_state_green && led_state_red){
            for (int vermelho = 0; vermelho < 8; vermelho++){ // Loop para o sinal vermelho (8 repetições de 2s)
                if (diurno){
                    pwm_set_gpio_level(BUZZER_A, 1);    
                    pwm_set_gpio_level(BUZZER_B, 1);    // Liga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(500));     // Aguarda 500ms
                    pwm_set_gpio_level(BUZZER_A, 0);    
                    pwm_set_gpio_level(BUZZER_B, 0);    // Desliga o buzzer A e B
                    vTaskDelay(pdMS_TO_TICKS(1500));    // Aguarda 1500ms
                    }
                else {
                    break;                              // Sai do loop se o modo for noturno
                    }
                }
            }
        }
    }
}

void vDisplayTask() // Task para controle do display
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char str_y[8]; // Buffer para armazenar a string
    bool cor = true;
    while (true)
    {
        snprintf(str_y, sizeof(str_y), "%s", diurno ? "Diurno" : "Noturno");    // Formata a string com o estado atual
        ssd1306_fill(&ssd, !cor);                                               // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);                           // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);                                // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);                                // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);                      // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);                       // Desenha uma string
        ssd1306_draw_string(&ssd, "  FreeRTOS", 10, 28);                        // Desenha uma string
        ssd1306_draw_string(&ssd, "Modo de Func.", 10, 41);                     // Desenha uma string
        ssd1306_draw_string(&ssd, str_y, 40, 52);                               // Desenha uma string
        ssd1306_send_data(&ssd);                                                // Atualiza o display
        vTaskDelay(pdMS_TO_TICKS(1000));                                        // Aguarda 1 segundo
    }
}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // Fim do trecho para modo BOOTSEL com botão B

    stdio_init_all();   // Inicialização Serial

    xTaskCreate(vSetStateTask, "Task Set State", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);       // Cria a task para o estado diurno/noturno
    xTaskCreate(vMatrizTask, "Task Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);            // Cria a task para a matriz de LEDs
    xTaskCreate(vLEDS_SemaforoTask, "Task Semáforo", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);   // Cria a task para o semáforo
    xTaskCreate(vBuzzerTask, "Task Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);            // Cria a task para o buzzer
    xTaskCreate(vDisplayTask, "Cont Task Disp3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);       // Cria a task para o display
    vTaskStartScheduler();
    panic_unsupported();
}
