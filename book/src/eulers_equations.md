# Newton's Three Laws
You've heard them time and time again, but we're about to use all 3!
    
1. An object at rest or in motion will remain at rest or in motion unless acted upon by a force. Basically just defines conservation of momentum (both linear and angular).
2. The acceleration of an object is proportional to the net external force acting on it and inversely proportional to its mass. Basically \\(F = ma\\), but more specifically "Force is the time rate of change of momentum."
3. For every action, there is an equal and opposite reaction.

# Euler's Equations of Motion
When it comes to defining atittude dynamics of spacecraft, we are specifically interested in the dynamics of a rigid body. Euler's equations of motion define rotation dynamics of a rigid body. They are:

\\[T = I\dot{\omega} + \dot{I}\omega + \omega \times H\\]

where 

\\(T\\) = torque in the body frame

\\(I\\) = body inertia tensor

\\(H\\) = angular momentum in the body frame = I\omega

To derive:

Newton's seconds law says that "Force is the time rate of change of momentum." However, Euler's equations of motion are derived in the body frame, since for a rigid body most quantities attitude quantities like angular rate, momentum, are expressed intuitively in the body's frame. Thus, we must apply the kinematic transport theorem to the derivative of our momentum
For rotation:
    \\[T = \frac{dH}{dt} + \omega \times H\\]
    \\[T = \frac{d}{dt}(I\omega)+ \omega \times H\\]
    \\[T = I\dot{\omega} + \dot{I}\omega+ \omega \times H\\]    

Note: Often \\(I\\) is constant so that \\(\dot{I} = 0\\) and that term cancels

The term \\(\omega \times H\\) is the familiar gyroscopic force. This can be 0 if the body frame is defined along the principal axes of the inertia tensor, and there are no internal momentum sources as we'll see in a sec. Both of these assumptions are very liekly not to be true for spacecraft.

It's important to remember that the body frame is only attached to the rigid body of the spacecraft, and does not account for the components or a total system frame.

# Euler's EOM with Internal Momentum
Euler's EOM account for a single rigid body. However, spacecraft tend to have a composition of a rigid body and many components. Many of the components are rotating themselves internal to the spacecraft body. For example, reaction wheels, solar arrays, instrument mechanisms. Note that even though solar arrays are physically external to the body, they are mechanically internal to the rotating system. 

From Newton's first law, angular momentum vector of the total system must be conserved in the inertial frame. Thus, if the momentum of a reaction wheel or solar array changes, the momentum of the rigid body will also change to maintain the momentum vector of the overall system.

Note that typically, there is no dynamic rotation between the component and body frames, they are typically rigidly attached. Thus, we do not need to account for kinematic transport theorem. (Note: this is not true for control moment gyros (CMG) or for example spinning mechanisms on a tilt!)

The internal momentum devices typically don't instantaneously change momentum, but are torqued over time. Newton's third law requires that whenever one of these rotating devices torque to change their momentum, they also produce an equal and opposite torque on the body, or an internal torque.

To derive this relationship:
\\[H_s = H_i + H_b \\] 
where 

\\(H_s\\) = system momentum in the body frame

\\(H_i\\) = internal momentum in the body frame

\\(H_b\\) = rigid body momentum in the body frame

\\[T_e = \frac{d}{dt}H_s + \omega \times H_s\\]    
\\[T_e = \frac{d}{dt}(H_i + H_b) + \omega \times (H_b + H_i)\\]
\\[T_e = \frac{d}{dt}H_i + \frac{d}{dt}H_b + \omega \times (H_b + H_i) \\]
\\[T_e = T_i + I\dot{\omega} + \dot{I}\omega + \omega \times (H_b + H_i) \\]
\\[T_e - T_i = I\dot{\omega} + \dot{I}\omega + \omega \times (H_b + H_i) \\]

This is the final equation of motion I produce when modeling a spacecraft. \\(T_e\\) is the external torque produced by environments and actuators. \\(T_i\\) is the equal and opposite torque applied by internal momentum components, and H_i is the momentum of those devices. 