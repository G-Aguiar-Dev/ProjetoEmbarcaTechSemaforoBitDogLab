# EmbarcaTechSemáforoBitDogLab
Projeto de desenvolvimento de um semáforo na BitDogLab, utilizando o sistema operacional FreeRTOS. Atividade proposta no programa de capacitação EmbarcaTech - TIC 37 - Fase 2

# Vídeo Demonstração



# Hardware/Firmware

Projeto desenvolvido em uma placa de desenvolvimento BitDogLab, versão 6.3.<br>
Desenvolvimento de firmware feito através do PicoSDK, com a IDE Visual Studio Code.

# Instruções

O semáforo desenvolvido possui dois modos de funcionamento, descritos no display:<br><br>

Diurno: O LED RGB presente na BitDogLab age como as luzes do semáforo. Para produzir o amarelo, os LEDs vermelho e verde são ligados ao mesmo tempo.<br><br>
A matriz de LEDs indica para o pedestre qual o sinal para atravessar a rua.<br><br>
Os buzzers são utilizados para indicar qual a cor atualmente presente no semáforo:<br><br>
Verde: 1 beep curto por um segundo;<br>
Amarelo: beep rápido intermitente;<br>
Vermelho: tom contínuo curto (500ms ligado e 1.5s desligado);<br><br>

Noturno: O LED RGB pisca em amarelo para indicar o modo de passagem livre, além do buzzer emitir sinais sonoros constantes indicando que a passagem de carros é livre.