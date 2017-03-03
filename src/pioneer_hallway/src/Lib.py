from collections import namedtuple

'''
 Given (state, action)
 Return (state)
 State is a namedtuple
   with (x,y,lin,rot)
   where x is x position
         y is y position
         lin is linear velocity
         rot is angular velocity
  when calling the function create a namedtuple
  with the appropriate fields filled in
  State = namedTuple("State", "x y lin rot")
      State.x | State.y | State.lin | State.rot
  action is a string "a0-a14" 
'''
def transition(state, action):
    (action_lin, action_rot) = lookUpPrimitiveAction(action)
    x = state.x
    y = state.y
    lin = state.lin
    rot = state.rot
    raise NotImplementedError

def lookUpPrimitiveAction(action):
    raise NotImplementedError
    
