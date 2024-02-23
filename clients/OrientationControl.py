import math 
class orientationControl:
    def __init__(self): 
        self.cumError = 0 
        
    def PID(self,heading,newHeading):
        w=0
        angleError = newHeading - heading
        if angleError > math.pi:
            angleError=angleError-2*math.pi
        elif angleError< (-math.pi):
            angleError=angleError+2*math.pi
        self.cumError= self.cumError+angleError

        if angleError<0 and  self.cumError>0:
            self.cumError=0   
        elif(angleError>0 and  self.cumError<0):
            self.cumError=0
            
        if( self.cumError>0):
            if( self.cumError>14):
                
                self.cumError=14 
        elif( self.cumError<0):

            if( self.cumError<-14):

                self.cumError=-14
        if(angleError>0.65 or angleError<-0.65):
              
            w=(0.4*angleError)+0.1*self.cumError#*self.SAMPLETIME/1000 
            print("w:")
            print(w)
        else:
            w=0
        return w
            