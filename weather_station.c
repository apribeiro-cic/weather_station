#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include "lwipopts.h"
#include "lib/html.h"
#include "lib/icons.h"
#include "weather_station.pio.h"
#include "pico/bootrom.h"

#include <math.h>

#include "lwip/pbuf.h"           // Lightweight IP stack - manipulação de buffers de pacotes de rede
#include "lwip/tcp.h"            // Lightweight IP stack - fornece funções e estruturas para trabalhar com o protocolo TCP
#include "lwip/netif.h"          // Lightweight IP stack - fornece funções e estruturas para trabalhar com interfaces de rede (netif)

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID "ID-DO-WI-FI"
#define WIFI_PASS "SENHA-DO-WI-FI"

#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define I2C_PORT i2c0                   // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                       // 0 ou 2
#define I2C_SCL 1                       // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0     // Pressão ao nível do mar em Pa

#define I2C_PORT1 i2c1              
#define I2C_SDA1 2
#define I2C_SCL1 3
#define endereco 0x3C

#define LED_BLUE_PIN 12                 // GPIO12 - LED azul
#define LED_GREEN_PIN 11                // GPIO11 - LED verde
#define LED_RED_PIN 13                  // GPIO13 - LED vermelho
#define BTN_A 5                         // GPIO5 - Botão A
#define BTN_B 6                         // GPIO6 - Botão B
#define BTN_J 22                        // GPIO22 - Botão Joystick
#define JOYSTICK_X 26                   // Pino do Joystick X
#define BUZZER_A_PIN 10                 // GPIO10 - Buzzer A
#define BUZZER_B_PIN 21                 // GPIO21 - Buzzer B

#define NUM_PIXELS 25   // Número de pixels da matriz de LEDs
#define MATRIX_PIN 7    // Pino da matriz de LEDs

uint32_t last_time = 0; // Variável para armazenar o tempo do último evento

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void start_http_server(void);
uint32_t matrix_rgb(double r, double g, double b); 
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b); 
bool checa_limites(float valor, float min, float max);
void pwm_setup_gpio(uint gpio, uint freq);
void gpio_bitdog(void); 
void gpio_irq_handler(uint gpio, uint32_t events); 
double calculate_altitude(double pressure); 

// Buffer para armazenar IP do Pico
static char ip_str[24];

struct http_state
{
    char response[8192];
    size_t len;
    size_t sent;
};

// Variáveis para controle de cor e ícone exibido na matriz de LEDs
double red = 0.0, green = 255.0 , blue = 0.0; // Variáveis para controle de cor
int icon = 0; //Armazena o número atualmente exibido
double* icons[2] = {icon_zero, icon_one }; //Ponteiros para os desenhos dos números

float temperatura, umidade, pressao, altitude;
float offset_temp = 0, offset_umid = 0, offset_press = 0;
float limite_temp_min = 0, limite_temp_max = 100;
float limite_umid_min = 0, limite_umid_max = 100;
float limite_press_min = 90, limite_press_max = 110;

bool current_sensor = true; // Variável para alternar entre os sensores AHT20 e BMP280 (true para AHT20, false para BMP280)

int main()
{
    stdio_init_all();
    gpio_bitdog();

    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_J, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    i2c_init(I2C_PORT1, 400 * 1000);

    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA1);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL1);                                        // Pull up the clock line

    //Configurações da PIO
    PIO pio = pio0; 
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, MATRIX_PIN);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT1);
    aht20_init(I2C_PORT1);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[5];  // Buffer para armazenar a string
    char str_alt[5];  // Buffer para armazenar a string  
    char str_tmp2[5];  // Buffer para armazenar a string
    char str_umi[5];  // Buffer para armazenar a string      

    bool cor = true;

    printf("Iniciando Wi-Fi...\n");

    // Aguarda alguns segundos para estabilização do Wi-Fi
    sleep_ms(2000);

    // Inicializa o Wi-Fi e o servidor web
    if (cyw43_arch_init())
    {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("Erro ao conectar ao Wi-Fi\n");
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    printf("Conectado ao Wi-Fi! IP: %s\n", ip_str);

    sleep_ms(500);

    start_http_server(); // Inicia o servidor HTTP

    while (true) {
        // Polling para manter conexão
        cyw43_arch_poll();  

        uint32_t current_time = to_us_since_boot(get_absolute_time());

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        altitude = calculate_altitude(pressure + offset_press * 1000.0); // Converte de kPa para Pa

        printf("Conectado ao Wi-Fi: %s\n", WIFI_SSID);
        printf("IP: %s\n", ip_str);

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT1, &data))
        {
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            printf("Erro na leitura do AHT10!\n\n\n");
        }

        sprintf(str_tmp1, "%.1fC", temperature / 100.0);  // Converte o inteiro em string
        sprintf(str_alt, "%.0fm", altitude);  // Converte o inteiro em string
        sprintf(str_tmp2, "%.1fC", data.temperature);  // Converte o inteiro em string
        sprintf(str_umi, "%.1f%%", data.humidity);  // Converte o inteiro em string        

        if (current_sensor) {
            temperatura = data.temperature + offset_temp; 
        } else {
            temperatura = temperature / 100.0 + offset_temp; 
        }

        umidade = data.humidity + offset_umid; // Umidade relativa em %
        pressao = pressure / 1000.0 + offset_press; // Converte de Pa para kPa

        if (checa_limites(temperatura, limite_temp_min, limite_temp_max) ||
            checa_limites(umidade, limite_umid_min, limite_umid_max) ||
            checa_limites(pressao, limite_press_min, limite_press_max)) {
            red = 255.0; green = 150.0; blue = 0.0; 
            gpio_put(LED_GREEN_PIN, 0); 
            gpio_put(LED_RED_PIN, 1); 
            icon = 1; 

            if (last_time == 0 || current_time - last_time >= 700000) { 
                pwm_setup_gpio(BUZZER_B_PIN, 1000); 
                sleep_ms(200); 
                pwm_setup_gpio(BUZZER_B_PIN, 0); 
                last_time = current_time;
            }
            
        } else {
            if (current_sensor) {
                red = 0.0; green = 0.0; blue = 255.0; 
            } else {
                red = 65.0; green = 15.0; blue = 135.0; 
            }

            gpio_put(LED_RED_PIN, 0); 
            gpio_put(BUZZER_B_PIN, 0);
            gpio_put(LED_GREEN_PIN, 1);
            icon = 0; 
        }

        desenho_pio(icons[icon], 0, pio, sm, red, green, blue);

        sleep_ms(300);
    }

    cyw43_arch_deinit(); 
    return 0;
}

// Função http callback para lidar com conexões TCP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Função de callback para novas conexões TCP e requisições HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "GET /estado")) {
        // JSON com os dados atuais e limites
        char json_payload[256];
        int json_len = snprintf(json_payload, sizeof(json_payload),
            "{"
            "\"temperatura\":%.2f,"
            "\"umidade\":%.2f,"
            "\"pressao\":%.2f,"
            "\"altitude\":%.2f,"
            "\"temp_min\":%.2f,\"temp_max\":%.2f,"
            "\"umid_min\":%.2f,\"umid_max\":%.2f,"
            "\"press_min\":%.2f,\"press_max\":%.2f,"
            "\"offset_temp\":%.2f,\"offset_umid\":%.2f,\"offset_press\":%.2f"
            "}",
            temperatura,
            umidade,
            pressao,
            altitude,
            limite_temp_min, limite_temp_max,
            limite_umid_min, limite_umid_max,
            limite_press_min, limite_press_max,
            offset_temp, offset_umid, offset_press
        );

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", json_len, json_payload);
    }
    else if (strstr(req, "GET /config?")) {
        // Extração de parâmetros da query
        char *min_temp = strstr(req, "temp_min=");
        char *max_temp = strstr(req, "temp_max=");
        char *min_umid = strstr(req, "umid_min=");
        char *max_umid = strstr(req, "umid_max=");
        char *min_press = strstr(req, "press_min=");
        char *max_press = strstr(req, "press_max=");
        char *off_temp = strstr(req, "offset_temp=");
        char *off_umid = strstr(req, "offset_umid=");
        char *off_press = strstr(req, "offset_press=");

        if (min_temp && max_temp && min_umid && max_umid && min_press && max_press) {
            limite_temp_min = atof(min_temp + 9);
            limite_temp_max = atof(max_temp + 9);
            limite_umid_min = atof(min_umid + 9);
            limite_umid_max = atof(max_umid + 9);
            limite_press_min = atof(min_press + 10);
            limite_press_max = atof(max_press + 10);

            if (off_temp) offset_temp = atof(off_temp + 12);
            if (off_umid) offset_umid = atof(off_umid + 12);
            if (off_press) offset_press = atof(off_press + 13);

            char json_config[256];
            int len = snprintf(json_config, sizeof(json_config),
                "{"
                "\"temp_min\":%.2f,\"temp_max\":%.2f,"
                "\"umid_min\":%.2f,\"umid_max\":%.2f,"
                "\"press_min\":%.2f,\"press_max\":%.2f,"
                "\"offset_temp\":%.2f,\"offset_umid\":%.2f,\"offset_press\":%.2f"
                "}",
                limite_temp_min, limite_temp_max,
                limite_umid_min, limite_umid_max,
                limite_press_min, limite_press_max,
                offset_temp, offset_umid, offset_press
            );

            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", len, json_config);
        } else {
            const char *erro = "Parametros ausentes ou invalidos";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 400 Bad Request\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", (int)strlen(erro), erro);
        }
    }
    else {
        // Página HTML principal
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);
    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

// Função de callback para aceitar novas conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função para iniciar o servidor HTTP
static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

// Rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double r, double g, double b) {
    unsigned char R, G, B;
    R = r * red;
    G = g * green;
    B = b * blue;
    return (G << 24) | (R << 16) | (B << 8);
}

// Rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b) {
    for (int16_t i = 0; i < NUM_PIXELS; i++) {
        valor_led = matrix_rgb(desenho[24-i], desenho[24-i], desenho[24-i]);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Função para verificar se um valor está fora dos limites
bool checa_limites(float valor, float min, float max) {
    return (valor < min || valor > max);
}

// Função para configurar o PWM
void pwm_setup_gpio(uint gpio, uint freq) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);  // Define o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);  // Obtém o slice do PWM

    if (freq == 0) {
        pwm_set_enabled(slice_num, false);  // Desabilita o PWM
        gpio_put(gpio, 0);  // Desliga o pino
    } else {
        uint32_t clock_div = 4; // Define o divisor do clock
        uint32_t wrap = (clock_get_hz(clk_sys) / (clock_div * freq)) - 1; // Calcula o valor de wrap

        // Configurações do PWM (clock_div, wrap e duty cycle) e habilita o PWM
        pwm_set_clkdiv(slice_num, clock_div); 
        pwm_set_wrap(slice_num, wrap);  
        pwm_set_gpio_level(gpio, wrap / 5);
        pwm_set_enabled(slice_num, true);  
    }
}

// Inicializar os Pinos GPIO da BitDogLab
void gpio_bitdog(void) {
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);

    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);

    gpio_init(BTN_J);
    gpio_set_dir(BTN_J, GPIO_IN);
    gpio_pull_up(BTN_J);

    gpio_init(BUZZER_A_PIN);  
    gpio_set_dir(BUZZER_A_PIN, GPIO_OUT);
    gpio_init(BUZZER_B_PIN);  
    gpio_set_dir(BUZZER_B_PIN, GPIO_OUT);
}

// Função de callback para tratamento de interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events) { 
    uint32_t current_time = to_us_since_boot(get_absolute_time()); // Pega o tempo atual em ms
    if (current_time - last_time > 250000) { // Debouncing de 250ms
        last_time = current_time;
        if (gpio == BTN_A) { // Verifica se o botão A foi pressionado 
            printf("Botão A pressionado!\n");
            current_sensor = !current_sensor; // Alterna entre os sensores AHT20 e BMP280
        } else if (gpio == BTN_B) { // Verifica se o botão B foi pressionado
            printf("Botão B pressionado!\n");
            reset_usb_boot(0, 0);
        } else if (gpio == BTN_J) {  // Verifica se o botão do joystick foi pressionado
            printf("Botão do joystick pressionado!\n");
        }
    }
}

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}