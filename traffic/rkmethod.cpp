#include<iostream>


#define f(x,y) (y*y-x*x)/(y*y+x*x) //equation to be solved

using namespace std;

class rk{
  
    public:
    float x0, y0, xn, h, yn, k1, k2, k3, k4, k; //enter values in matrix
    int i, n; //define variables for the loop
    void set_h(){    
        h = (xn-x0)/float(n); //step size->defines precision of the algorithm
    }
    
    void get_variables(){
        
        
        /*important functions to calculate step variables*/
        for(i=0; i < n; i++)
        {
            k1 = h * (f(x0, y0));
            k2 = h * (f((x0+h/2), (y0+k1/2)));
            k3 = h * (f((x0+h/2), (y0+k2/2)));
            k4 = h * (f((x0+h), (y0+k3)));
            k = (k1+2*k2+2*k3+k4)/6;
            yn = y0 + k;
            cout<< x0<<"\t"<< y0<<"\t"<< yn<< endl;
            x0 = x0+h;
            y0 = yn;
        }
        
        cout<<"\nValue of y at x = "<< xn<< " is " << yn;
}
};

int main()
{
    rk Rkp1;
   
    cout<<"Enter Initial Condition"<< endl;
    cout<<"x0 = ";
    cin>> Rkp1.x0;
    cout<<"y0 = ";
    cin >> Rkp1.y0;
    cout<<"Enter calculation point xn = "; //no of midpoints rk will calculate at. this enhances precision of answers.
    cin>>Rkp1.xn;
    cout<<"Enter number of steps: ";
    cin>> Rkp1.n;

    



    cout<<"\nx0\ty0\tyn\n"; //shows the variable value entered
    cout<<"------------------\n"; //space
 
    Rkp1.set_h();
    Rkp1.get_variables();
  
    return 0;

  
 }

 
 

