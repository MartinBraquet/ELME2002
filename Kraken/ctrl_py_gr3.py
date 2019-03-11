
class PyControl:
    
    # constructor call at initialization
    def __init__(self, inputs):
        print('Python controller created for Gr3')
    
    # loop function
    def loop_PyController(self, args):
        # Retrieve the inputs
        inputs = args[0]
        
        # Here compute the outputs
        t = inputs["t"]
        wcR = 2*t
        wcL = -2*t

        # Return the computed outputs
        outputs = { "wheel_commands": (wcR, wcL), 
                    "tower_command": 10, 
                    "flag_release": 0}

        return outputs
    
    # closing function    
    def close_PyController(self):
        print('Closing Python controller')

