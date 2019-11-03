# Projeto Assembly ARM utilizando a Beaglebone

 1. Introdução
 2. Leitura dos números e da operação
 3. Soma normal
 4. Soma Modular
 5. Divisão
 6. Multiplicação
 7. Rotação
 8. Reset	

## 1. Introdução

Este trabalho consiste em implementar uma calculadora usando os LEDs da placa (usr0, usr1, usr2 e usr3) para apresentar o resultado em binário. Foi implementada as seguintes funções:

**Soma** - recebe dois números inteiros e mostra o resultado da soma entre eles. Obs.: se o resultado não for representável por 4 bits, deve-se mostrar o erro somaERROR.

**SomaModular** - recebe dois números inteiros e mostra o resultado da soma entre eles módulo 16.

**MultiplicacaoModular** - recebe dois números inteiros e mostra o resultado da multiplicação entre eles módulo 16.

**Rotação** - recebe um número de 4 bits x e um valor n, indicando quantos bits o x deve ser rotacionado e mostra o resultado de x rotacionado n vezes.

**Divisão** - recebe dois números inteiros (x e y) mostra o resultado da divisão inteira de x por y. Obs.: se o resultado não for representável por 4 bits, deve-se mostrar o erro divisaoERROR.

Caso aconteça algum desses erros, vai seguir o padrão abaixo e voltar para o modo inicial.

**somaERROR** - pisca os LEDs usr0 e usr3 três vezes.

**divisaoERROR** - pisca os LEDs usr1 e usr2 três vezes.

**outros** - pisca os quatro LEDs 5 vezes.

Esses outros erros seriam as operações com operandos inválidos, por exemplo: quando o usuário digitar uma string em vez de um número.

## 2. Leitura dos números e da operação

Primeiro entraremos na função .scanfdoassembly e pegamos um carácter, nesse momento ainda não sabemos se é número ou string, então comparamos com 32 para saber se é um espaço, pois quando reconhecer um espaço ele considera ter pego um carácter e irá pegar o próximo carácter. Bem, mas porque comparamos com o 32? pois de acordo a tabela ASCII o 32 em decimal é equivalente ao espaço.

Então vamos, explicar melhor com um exemplo: suponha que eu digite 1, quando ele comparar com o espaço (32) não será igual e será printado no terminal o valor que digitei no caso 1. E vou fazer mais duas comparações se for maior que 57 e menor que 48 não é um número, pois os números da tabela ASCII se encontram só nesse intervalo.

depois faremos a seguinte operação sub r2,r2,#48 para sabermos o número que foi digitado, e ter certeza que é um número. Logo em seguida multiplicamos por dez e salvaremos o nosso primeiro dígito em hexadecimal no registrador r11. E voltamos para o início da função  .scanfdoassembly agora digitamos no terminal o espaço e como ele é igual ao 32 vai entrar na função .proximodoscanfdoassembly no qual prepara para pegar o próximo caracter. Essa função salvará o que tem no r11 guardando no r9 e depois limpa com zero o r11 para esperar o próximo caractere, para quando eu repetir o procedimento digitando o próximo número ele ficar guardado no r9.

Logo depois de digitar os operandos digitamos a operação e precisamos identificar qual operação desejamos realizar. Para isso, apenas comparamos qual operação ele (usuário) escolheu e encaminhamos para a função da operação.

```
    cmp r0,'+'
    bleq .somaNormal
 
    cmp r0,'='   
    bleq .SomaModular
 
    cmp r0,'/'
    bleq .divisao
 
    cmp r0,'*'
    bleq .multiplicacao
 
    cmp r0,'<'
    bleq .deslocaparaesquerda

```

## 3. Soma normal

Começa entrando na função .somaNormal e logo em seguida irá para a função .soma_normal e lá fará a operação de soma que é apenas um add, a nossa função mais simples e como termina com ‘bx lr’ ela volta para onde foi chamada e ai comparamos com 15, caso o valor seja maior que 15 não vamos conseguir representar ele com apenas 4 bits e vamos para a funçao .errosoma, nessa função vamos piscar três vezes como o professor pede. E caso não for um erro ele irá para a função .resultadoleds que imprime os leds com o resultado, damos o shift 21 vezes, pois o led da plca é apartir do 21, ai ele fica certinho nos led da placa. A gente pega o r11 e joga no ofsset dos leds e salvamos. Depois vamos para a função reset, na qual ficará esperando o botão ser acionado para resetar e começar um novo processo.

## 4. Soma Modular

Vamos começar entrando na função .SomaModular faremos uma soma, nada de mais, e entramos no loop no qual faremos o mod 16, primeiro comparamos com 16 caso ele seja menor, então entra na função somaMenor e como finaliza com ‘bx lr’ volta para onde foi chamada e caso não seja menor que 16 faremos um sub  r11,r11,#16 para obter o módulo de 16 e se continuar maior ou igual continua no loop senão ele entra na função somaMenor e como finaliza com ‘bx lr’ volta para onde foi chamada e entrará na função resultado dos leds. Depois vamos para a função reset, na qual ficará esperando o botão ser acionado para resetar e começar um novo processo.

## 5. Divisão

Começaremos chamando a função .divisao a primeira coisa que faremos é tratar o erro, pois está escrito: “Não dividirás por zero - primeiro mandamento da matemática” então vamos comparar r11 que é o nosso último número a ser pegado e vamos verificar se ele é zero, se for zero vai para a função de erro e irá piscar três vezes como o professor pede. Caso não seja zero, iremos para a função .divi_semerro e vamos limpar o r3, pois lá estará o nosso quociente. Não vamos nos importar com os restos nessa divisão então só faremos o seguinte: subs  r9,r9,r11 e addge r3,r3,#1 dentro do loop até r9 ser maior e igual ao r11, neste caso iremos mover o r3 no qual está com a resposta para o r11 e como terminamos com ‘bx lr’ voltaremos para onde a função .divi_semerro foi chamada e depois entrará na função resultado dos leds. Depois vamos para a função reset, na qual ficará esperando o botão ser acionado para resetar e começar um novo processo.

## 6. Multiplicação

Vamos começar entrando na função .multiplicacao vamos limpar o registrador r7 e moveremos os r9 (que foi o último número que digitamos para a operação) para o r4 e vamos fazer a multiplicação:

```
  .loop_multiplica:
    add r7,r7,r11
    subs r4,r4,#1
    bne .loop_multiplica
 
    mov r11,r7

```
a lógica é somar o r11, r9 vezes. O resultado ficará em r7, vamos mover o r7 para o r11, apenas para deixar o r11 com o resultado (isto não é necessário, só é necessário depois de passar pelo mo 16. Mas fiz assim para ficar padrão com todas as respostas em r11). Depois entra no mod 16, a lóica é bem parecida com o mod da soma modular. A única coisa que muda é que após ficar menor que 16 eu farei o seguinte: add r4,r11,#16 Assim o resultado da multiplicação com mod16 estará no r4, porém precisamos mover para o r11, após mover para o r11 ela voltará para onde a função foi chamada, devido ao ‘bx lr’. Depois vamos para a função reset, na qual ficará esperando o botão ser acionado para resetar e começar um novo processo.

## 7. Rotação

Bem, o meu deslocamento é feito somente para a esquerda. E começamos entrando na função .deslocaparaesquerda nessa função optei por mover alguns registradores para depois começar a trabalhar neles, como são coisas óbvias vou explicar apenas a lógica. Dentro do loop, com o valor que recebemos vamos deslocar uma vez para a esquerda e mover para o r3, então peguei o r3 e fiz um deslocamento para esquerda 4 vezes. E então, faço uma or com r3 e r1 para pegar os valores setados e depois faço uma and com quinze, pois no quinze todos os bits estão setados, quando eu termino de fazer isso, faço a subtração de um com o valor de vezes que quero rotacionar, quando chegar a zero termino o deslocamento e coloco o resultado no r11. Após mover para o r11 ela voltará para onde a função foi chamada, devido ao ‘bx lr’. Depois vamos para a função reset, na qual ficará esperando o botão ser acionado para resetar e começar um novo processo.

## 8. Reset

A função reset tem como papel recomeçar a função da calculadora, e fazemos isso com um botão. Ao entrar na função .botão passamos o GPIO1_BASE e o GPIO1_DATAIN para o r0. Para saber o endereço dessas constantes, bastou acessar o datasheet. Enviamos todo conteúdo de r0 para o r1 e deslocamos 1 << 15 para limpar todos os bits para ficar somente o bit 15. Em seguida deslocamos o r1 para direita: lsr r1,r1,#15 e por fim comparamos o r1 com 1. Se for igual, resetamos a nossa calculadora. E caso não seja igual, eu fico no loop .botão.  E como daremos um reset vamos terminar a função reset pulando para a main, com a instrução ‘b .main’



![](https://www.codemade.io/wp-content/uploads/2016/05/robot-controlled-using-htmljavascript-and-beaglebone-green-hacksterio-146304747348nkg.jpg)