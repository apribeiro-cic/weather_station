# Estação Meteorológica com Interface Web 🌤️

Este projeto foi desenvolvido para a placa BitDogLab, baseada no microcontrolador Raspberry Pi Pico W, e tem como objetivo monitorar em tempo real variáveis ambientais como temperatura, umidade, pressão atmosférica e altitude.

Além da exibição local por periféricos embarcados (OLED, matriz de LEDs, LED RGB, buzzer), o sistema conta com uma interface web responsiva, acessível por navegador em qualquer dispositivo conectado à mesma rede Wi-Fi.

---

## 📌 Sobre o Projeto

A Estação Meteorológica foi criada como projeto prático da 2ª fase da residência tecnológica EmbarcaTech.

Ela demonstra o uso integrado de diversos recursos da BitDogLab, como GPIOs, ADC, I²C, PIO, PWM e Wi-Fi com pilha TCP/IP (LWIP), implementando um servidor HTTP local completo, com respostas em HTML, JSON e controle de estado.

---

## 🧠 Como funciona

O sistema realiza leituras periódicas de sensores ambientais e disponibiliza os dados tanto localmente quanto remotamente:

🧪 Leitura de Sensores
 - Sensor AHT20 (I²C): lê temperatura e umidade.

 - Sensor BMP280 (I²C): lê temperatura e pressão, além de estimar a altitude baseada na pressão atmosférica.

 - A leitura de temperatura ativa depende do sensor selecionado com o botão A.

🌐 Interface Web

A página HTML é servida diretamente pelo microcontrolador, exibindo:

 - Dados ao vivo (temperatura, umidade, pressão, altitude).

 - Gráficos dinâmicos via JavaScript, atualizados a cada segundo via GET /estado.

 - Formulário de ajuste de limites e offsets por sensor (enviado por GET /config?...).

📊 Visualização local (na placa)

 - OLED: mostra IP da rede, sensor ativo, e valores atuais.

 - Matriz de LEDs WS2812B: exibe ícones coloridos representando sensor ativo ou status de alerta.

 - LED RGB:

    - Verde = todos os dados dentro dos limites.

    - Vermelho = algum dado fora dos limites.

 - Buzzer: emite alerta sonoro sempre que qualquer leitura ultrapassa os limites definidos.

### 🔄 🎮 Controles


Controle	Função
Botão A	Alterna o sensor ativo (AHT20 ou BMP280)
Botão B	Reinicia a placa em modo BOOTSEL (via interrupção com debounce)
Formulário Web	Permite ao usuário configurar min, max e offset por parâmetro

---

## 📁 Utilização

Atendendo aos requisitos de organização da 2ª fase da residência, o arquivo CMakeLists.txt está configurado para facilitar a importação do projeto no Visual Studio Code. 

Segue as instruções:

1. Na barra lateral, clique em **Raspberry Pi Pico Project** e depois em **Import Project**.

   ![image](https://github.com/user-attachments/assets/4b1ed8c7-6730-4bfe-ae1f-8a26017d1140)

2. Selecione o diretório do projeto e clique em **Import** (utilizando a versão **2.1.1** do Pico SDK).

   ![image](https://github.com/user-attachments/assets/be706372-b918-4ade-847e-12706af0cc99)

3. **IMPORTANTE**! Para o código funcionar é necessário trocar os parâmetros de SSID e SENHA do Wi-Fi para os da sua rede local.


4. Agora, basta **compilar** e **rodar** o projeto, com a placa **BitDogLab** conectada.

---
