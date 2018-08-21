
#include "stm32f10x_conf.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

#define trimf_c 0
#define gaussmf_c 1
#define trapmf_c 2

class Fuzzy_controller
{
public:
    const static int N=7;//������������ģ���Ӽ��ĸ���
private:
    float target;//ϵͳ�Ŀ���Ŀ��
    float actual;//������õ�ʵ��ֵ
    float e;     //���
    float e_pre; //��һ�ε����
    float de;    //���ı仯��
    float emax;  //��������������
    float demax; //���绯�ʻ������������
    float umax;  //���������
    float Ke;    //Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Kde;   //Ke=n/demax,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Ku;    //Ke=umax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
    int rule[N][N];//ģ�������
    int mf_t_e;   //e�������Ⱥ�������
    int mf_t_de;  //de�������Ⱥ�������
    int mf_t_u;   //u�������Ⱥ�������
    float *e_mf_paras; //���������Ⱥ����Ĳ���
    float *de_mf_paras;//����ƫ�������Ⱥ����Ĳ���
    float *u_mf_paras; //����������Ⱥ����Ĳ���
		int error;
public:
    Fuzzy_controller(float e_max,float de_max,float u_max);
    ~Fuzzy_controller();
    float trimf(float x,float a,float b,float c);          //���������Ⱥ���
    float gaussmf(float x,float ave,float sigma);          //��̬�����Ⱥ���
    float trapmf(float x,float a,float b,float c,float d); //���������Ⱥ���
    //����ģ�������Ⱥ����Ĳ���
    void setMf(const int & mf_type_e,float *e_mf,const int & mf_type_de,float *de_mf,const int & mf_type_u,float *u_mf);
    void setRule(int rulelist[N][N]);                          //����ģ������
    float realize(float t,float a);              //ʵ��ģ������

};


Fuzzy_controller::Fuzzy_controller(float e_max,float de_max,float u_max):
target(0),actual(0),emax(e_max),demax(de_max),umax(u_max),e_mf_paras(NULL),de_mf_paras(NULL),u_mf_paras(NULL)
{
   e=target-actual;
   e_pre=0;
   de=e-e_pre;
   Ke=(N/2)/emax;
   Kde=(N/2)/demax;
   Ku=umax/(N/2);
   mf_t_e=trimf_c;
   mf_t_de=trimf_c;
   mf_t_u=trimf_c;
}

Fuzzy_controller::~Fuzzy_controller()
{
  delete [] e_mf_paras;
  delete [] de_mf_paras;
  delete [] u_mf_paras;
}
//���������Ⱥ���
float Fuzzy_controller::trimf(float x,float a,float b,float c)
{
   float u;
   if(x>=a&&x<=b)
       u=(x-a)/(b-a);
   else if(x>b&&x<=c)
       u=(c-x)/(c-b);
   else
       u=0.0;
   return u;

}

//��̬�����Ⱥ���
float Fuzzy_controller::gaussmf(float x,float ave,float sigma) 
{
    float u;
    if(sigma<0)
    {
       //cout<<"In gaussmf, sigma must larger than 0"<<endl;
			error=0;
    }
    u=exp(-pow(((x-ave)/sigma),2));
    return u;
}


//���������Ⱥ���
float Fuzzy_controller::trapmf(float x,float a,float b,float c,float d)
{
    float u;
    if(x>=a&&x<b)
        u=(x-a)/(b-a);
    else if(x>=b&&x<c)
        u=1;
    else if(x>=c&&x<=d)
        u=(d-x)/(d-c);
    else
        u=0;
    return u;
}

//����ģ������
void Fuzzy_controller::setRule(int rulelist[N][N])
{
    for(int i=0;i<N;i++)
       for(int j=0;j<N;j++)
         rule[i][j]=rulelist[i][j];
}

//"trimf"=0
//"gaussmf"=1
//"trapmf"=2
//����ģ�������Ⱥ��������ͺͲ���
void Fuzzy_controller::setMf(const int &  mf_type_e,float *e_mf,const int  & mf_type_de,float *de_mf,const int  & mf_type_u,float *u_mf)
{
    if(mf_type_e==trimf_c||mf_type_e==gaussmf_c||mf_type_e==trapmf_c)
        mf_t_e=mf_type_e;
    else
        //cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
				error=1;
    if(mf_type_de==trimf_c||mf_type_de==gaussmf_c||mf_type_de==trapmf_c)
        mf_t_de=mf_type_de;
    else
        //cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
				error=2;
    if(mf_type_u==trimf_c||mf_type_u==gaussmf_c||mf_type_u==trapmf_c)
        mf_t_u=mf_type_u;
    else
        //cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
				error=3;
    e_mf_paras=new float [N*3];
    de_mf_paras=new float [N*3];
    u_mf_paras=new float [N*3];
    for(int i=0;i<N*3;i++)
       e_mf_paras[i]=e_mf[i];
    for(int i=0;i<N*3;i++)
       de_mf_paras[i]=de_mf[i];
    for(int i=0;i<N*3;i++)
       u_mf_paras[i]=u_mf[i];
}


//ʵ��ģ������
float Fuzzy_controller::realize(float t,float a)   
{
    float u_e[N],u_de[N],u_u[N];
    int u_e_index[3],u_de_index[3];//����һ��������༤��3��ģ���Ӽ�
    float u;
    int M;
    target=t;
    actual=a;
    e=target-actual;
    de=e-e_pre;
    e=Ke*e;
    de=Kde*de;
    if(mf_t_e==trimf_c)
        M=3;               //���Ǻ�������������
    else if(mf_t_e==gaussmf_c)
        M=2;              //��̬��������������
    else if(mf_t_e==trapmf_c)
        M=4;              //���κ������ĸ�����
    int j=0;
    for(int i=0;i<N;i++)
    {
        u_e[i]=trimf(e,e_mf_paras[i*M],e_mf_paras[i*M+1],e_mf_paras[i*M+2]);//eģ��������������������
        if(u_e[i]!=0)
            u_e_index[j++]=i;                                              //�洢�������ģ���Ӽ����±꣬���Լ�С������
    }
    for(;j<3;j++)u_e_index[j]=0;

    if(mf_t_e==trimf_c)
        M=3;              //���Ǻ�������������
    else if(mf_t_e==gaussmf_c)
        M=2;              //��̬��������������
    else if(mf_t_e==trapmf_c)
        M=4;               //���κ������ĸ�����
    j=0;
    for(int i=0;i<N;i++)
    {
        u_de[i]=trimf(de,de_mf_paras[i*M],de_mf_paras[i*M+1],de_mf_paras[i*M+2]);//deģ��������������������
        if(u_de[i]!=0)
            u_de_index[j++]=i;                                                    //�洢�������ģ���Ӽ����±꣬���Լ�С������
    }
    for(;j<3;j++)u_de_index[j]=0;

    float den=0,num=0;
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
           num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*rule[u_e_index[m]][u_de_index[n]];
           den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
        }
    u=num/den;
    u=Ku*u;
    if(u>=umax)   u=umax;
    else if(u<=-umax)  u=-umax;
    e_pre=e;
    return u;
}
