#include <iostream>
using namespace std;

float distance (int b)
{
	float c;
if(b==0x0001)
	c=8.5;
else if(b==0x8000)
	c=-8.5;
else if(b==0x0003)
	c=7.5;
else if(b==0xC000)
	c=-7.5;
else if(b==0x0007)
	c=6.5;
else if(b==0xE000)
	c=-6.5;
else c=0;
return c;
}
void main()
{
	
	cout<<"������AGV���ص�����"<<endl;
	int a;
	cin>>hex>>a;
   
	cout<<"ƫ��"<<distance(a)<<"cm";
    system("pause");
}  