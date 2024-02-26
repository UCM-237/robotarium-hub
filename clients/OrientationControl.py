import math 
class orientationControl:
    def __init__(self, sample_time, kp=0.4, ki=0.1, kd=0.1, windup_limit=14, derivative_filter=False, filter_coeff=0.5): 
        self.cumError = 0
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.previousError=0
        self.windup_limit=windup_limit
        self.sample_time=sample_time/1000.0 #in seconds
        self.derivative_filter=derivative_filter
        self.filter_coeff=filter_coeff
        self.minimum_error=0.628 #two discrete steps of the wheels
        self.previousDerivative=0
        
    def PID(self,heading,newHeading):
        w=0
        angleError = newHeading - heading
        
        if angleError > math.pi:
            angleError=angleError-2*math.pi
        elif angleError< (-math.pi):
            angleError=angleError+2*math.pi
        
        self.cumError= self.cumError+angleError
        #antiwindup and reset for integral part
        if angleError<0 and  self.cumError>0:
            self.cumError=0   
        elif(angleError>0 and  self.cumError<0):
            self.cumError=0
            
        if( self.cumError>0):
            if( self.cumError>self.windup_limit):
                
                self.cumError=self.windup_limit 
        elif( self.cumError<0):

            if( self.cumError<-self.windup_limit):

                self.cumError=-self.windup_limit
        derivative = (angleError - self.previousError) /  self.sample_time
        #derivative filter
        if self.derivative_filter:
            derivative = self.filter_coeff * derivative + (1 - self.filter_coeff) * self.previousDerivative
            self.previousDerivative = derivative
            
        if(angleError>self.minimum_error or angleError<-self.minimum_error):
        
            w=(self.kp*angleError)+self.ki*self.cumError* self.sample_time + self.kd*derivative
            print("w:",w)
        
        else:
            w=0
        return w
            