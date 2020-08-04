#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/supervisor.h>
#include <webots/inertial_unit.h>

#define TIME_STEP 64
//define aqui uma função para pegar o max entre dois números
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define M_PI 3.14159265358979323846

void delay(int time_milisec)
{
double currentTime, initTime, Timeleft;
double timeValue = (double)time_milisec/1000;
initTime = wb_robot_get_time();
Timeleft =0.00;
while (Timeleft < timeValue)
 {
  currentTime = wb_robot_get_time();
  Timeleft=currentTime-initTime;
  wb_robot_step(TIME_STEP);
  }
}
//código para zerar a matriz 
void zerarMatriz(double gridMap[16][16]){

 for(int i=0;i<16;i++){
 
   for(int j=0;j<16;j++){
   
     gridMap[i][j]=0;
   
   }
 
 }

}
//código que printa a matriz no console da simulação
void mostrarMatriz(double gridMap[16][16]){

  for(int i=1;i<16;i++){
 
   for(int j=1;j<16;j++){
   
       printf("%.2f ",gridMap[i][j]);
   
   }
 
    printf("\n");
 }
 
  printf("----------------------------------------------\n");
  printf("----------------------------------------------\n");
  printf("----------------------------------------------\n");

}
//código para converter as coordenadas do mundo para as coordenadas da matriz
void converterCoordenadasGrid(double i,double j,double x,double y,double gridMap[16][16],double dist){

     if(dist < 5){

         int range=1;//distância para ir depois de atingir o objetivo
         int cheguei=0;
         int posi=round(8+i);//converte as coordenadas do carinho para a matriz
         int posj=round(8-j);//idem
         int indicei = round((8+i)+x);//soma a coordenada do carrinho até a posição x do objeto encontrado pelo sensor de distancia
         int indicej = round((8-j)+y);// "                                         "y "                                           "
     
         //nesse for ele itera até a posição do objeto iterando nos vetores decompostos, para encontrar o caminho antes e depois de encontrar
         //o objeto atingido pelo sensor
         for(double k=0;k<=5;k=k+0.2){
         
           int subi = posi+round(k*x);//aqui é onde se anda pelos vetores decompostos até a posição do objeto encontrado
           int subj = posj+round(k*y);
         
           //aqui só verifica se a posição na hora de converter esta dentro do limite da matiz estabelecida
           //como o sensor tem uma série de ruídos, então pode acontecer de retornar uma posição -1 na hora da conversão  
           if(subi>0 && subi<16 && subj>0 && subi<16){
           
             //variável que verifica se já chegou na posição do objeto final
             if(cheguei==0){
             
                //se ainda não chegou, verifica se chegou, e se chegou seta a variavel para 1 dizendo que ja encontrou a posição alvo
                if(subi==indicei && subj==indicej){
                  cheguei=1;
                  //se encontrou, então seta a probabilidade para log de 0.6
                  gridMap[subi][subj]+=log10(0.6/0.4);
                  
                }else{
                    
                    //senão encontrou, então seta log de 0,3 para as celulas do caminho
                    gridMap[subi][subj]+=log10(0.3/0.7);
                 }
             
             }else{
                //caso chegou seja 1, então já encontrou a posição algo, então seguindo o grafico apresentado na video aula e a tabela
                //em um range de 1, ele seta 0.8 como probabilidade
                if(abs(subi-indicei)<=range && abs(subj-indicej)<=range && 
                subi>0 && subi<16 && subj>0 && subi<16){
                  gridMap[subi][subj]+=log10(0.8/0.2);
                  
                }
             
             }
           }
           
         }
         
     }

}

//transforma grau -> rad
double calculaAngulo(double ang){

   return (M_PI*ang)/180;

}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  //instanciando motores
  WbDeviceTag left_motor_1 = wb_robot_get_device("front left wheel");
  WbDeviceTag right_motor_1 = wb_robot_get_device("front right wheel");
  WbDeviceTag left_motor_2 = wb_robot_get_device("back left wheel");
  WbDeviceTag right_motor_2 = wb_robot_get_device("back right wheel");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag yaw = wb_robot_get_device("inertial unit");
  
  //instanciando todos  sensores da frente
  WbDeviceTag sensf4 = wb_robot_get_device("so4");
  WbDeviceTag sensf6 = wb_robot_get_device("so6");
  WbDeviceTag sensf7 = wb_robot_get_device("so7");
  WbDeviceTag sensf5 = wb_robot_get_device("so5");
  
  //aplicando o tempo em passos que os sensores vão caputar sinais
  wb_distance_sensor_enable(sensf4,TIME_STEP);
  wb_distance_sensor_enable(sensf6,TIME_STEP);
  wb_distance_sensor_enable(sensf7,TIME_STEP);
  wb_distance_sensor_enable(sensf5,TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_inertial_unit_enable(yaw, TIME_STEP);
 
  //setando posição do motor
   wb_motor_set_position(left_motor_1,INFINITY);
   wb_motor_set_position(right_motor_1,INFINITY);
   wb_motor_set_position(left_motor_2,INFINITY);
   wb_motor_set_position(right_motor_2,INFINITY);
  
  //velocidade dos motores nas 4 rodas
  double left_speed_1 = 0;
  double left_speed_2 = 0;
  double right_speed_1=0;
  double right_speed_2=0;
  
  //variaveis que vão pegar os valores dos sensores
  //double sensf2_value;
  double sensf4_value;
  double sensf6_value;
  double sensf7_value;
  double sensf5_value;
  //variáveis de controle 
  double position=200;
  double old_error=0;
  //constantes
  double p_gain = 0.5; 
  double d_gain = 0.55;

  //instania a matriz
  double gridMap[16][16];
  //a matriz é zerada
  zerarMatriz(gridMap);


  while (wb_robot_step(TIME_STEP) != -1) {
  
     
     double *valores = wb_inertial_unit_get_roll_pitch_yaw(yaw);
 
     //variáveis para o cálculo do angulo apresentado no relatório
     double anguloDiff1 = -valores[2]-calculaAngulo(10);
     double anguloDiff4 = -valores[2]-calculaAngulo(70);
     
     //pega as coordenadas do gps
     double *coord=wb_gps_get_values(gps);
     
     //pegando os valores dos sensores
     sensf4_value = wb_distance_sensor_get_value(sensf4);
     sensf6_value = wb_distance_sensor_get_value(sensf6);
     sensf7_value = wb_distance_sensor_get_value(sensf7);
     sensf5_value = wb_distance_sensor_get_value(sensf5);

      //convertendo o valor do sensor para metros, como explicado no relatório     
     double distanciaMetros1=5-(5*sensf4_value)/1024;
     double distanciaMetros4=5-(5*sensf7_value)/1024;
    
  
     //aqui é realizada a decomposição do vetor resultante para o sensor 4  
     double andarY1=cos(anguloDiff1)*distanciaMetros1;
     double andarX1=sin(anguloDiff1)*distanciaMetros1;
     
     //idem só que para o sensor 7
     double andarY4=cos(anguloDiff4)*distanciaMetros4;
     double andarX4=sin(anguloDiff4)*distanciaMetros4;
     //aqui faz o cáculo das probabilidades e logo depois imprime a matriz resultante
     converterCoordenadasGrid(coord[0],coord[2],andarX1,andarY1,gridMap,distanciaMetros1);
     converterCoordenadasGrid(coord[0],coord[2],andarX4,andarY4,gridMap,distanciaMetros4);
     mostrarMatriz(gridMap);
      
     //verifica se há algum objeto na frente de acordo com o valor retornado pelos sensores
     if(sensf4_value > 850 || sensf5_value >850){
     
         //seta velocidades contrárias para rotacionar o robô
         left_speed_1 = -3.0;
         left_speed_2 = -3.0;
         right_speed_1= 3.0;
         right_speed_2= 3.0;
     
         //seta as velocidades
          wb_motor_set_velocity(left_motor_1,left_speed_1);
          wb_motor_set_velocity(left_motor_2,left_speed_2);
          wb_motor_set_velocity(right_motor_1,right_speed_1);
          wb_motor_set_velocity(right_motor_2,right_speed_2);
     
         //aplica um delay de 0,2s
         delay(200);
         //volta a velocidade ao normal
         left_speed_1 = 3.0;
         left_speed_2 = 3.0;
         right_speed_1= 3.0;
         right_speed_2= 3.0;
     
     }
    
     //aqui é feita a verificação entre os sensores da frente e o do lado direito, para então ser pego o maior valor. O maior valor entre eles é o que importa, pois isso significa que tem alguma 
     //chance de colisão     
     double dist = (1024.0-MAX(sensf7_value,sensf6_value));
     double error = position - dist;
     //aqui é feito uma janela para o cálculo da integral, sendo essa janela de 50 iterações, ou seja, uma iteração de 50 time step faz o calculo com a integral, depois outros 50 não considera
     //isso tira os altos erros acumulados   
       
     //aqui é o código do slide, não mudei nada
     double dif_erro= error - old_error; 
     old_error = error;
     double power = p_gain*error + d_gain*dif_erro;
          
     right_speed_1 = 3.0+power;   
     right_speed_2 = 3.0+power;   
         
     if(right_speed_1 > 5.0){
         
        right_speed_1 = 5.0;
        right_speed_2 = 5.0;
         
      }
      if(right_speed_1 < 1.0){
         
        right_speed_1 = 1.0;
        right_speed_2 = 1.0;
         
       }
     
       //aqui seta a velocidade dos motores
       wb_motor_set_velocity(left_motor_1,left_speed_1);
       wb_motor_set_velocity(left_motor_2,left_speed_2);
       wb_motor_set_velocity(right_motor_1,right_speed_1);
       wb_motor_set_velocity(right_motor_2,right_speed_2);     
     

  };

  wb_robot_cleanup();

  return 0;
}