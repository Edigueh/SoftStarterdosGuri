# SoftStarterdosGuri

Pra comunicar usando o a USART2(PC) muda

int rx[5]; //Tem q ter esse tamanho na declaração

//E no Callback da USART2 tem q fzer esse esquema

if(rx[4] == 13) // se o Usuário apertou enter
{
    tempo_aceleracao = (((rx[0]-'0')*10) + (rx[1]-'0')); // dois primeiros bits são o tempo de subida
    tempo_desaceleracao = (((rx[2]-'0')*10) + (rx[3]-'0')); // dois ultimos bits são o tempo de descida
    //e o resto fica igual praticamente, fzer os transmit, msg e etc
}
