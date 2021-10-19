# element_weight-retargeting

This repository contains the application retargeting the weight of objects manipulated by the robotic avatar towards the human operator.

## Responsible:
|     [Riccardo Grieco](https://github.com/RiccardoGrieco)|
|-----------------------------------------------------------| 
|<img src="https://github.com/RiccardoGrieco.png" width="180">|

## Background
We are participating to the [ANA Avatar XPrize](https://www.xprize.org/prizes/avatar) competition, which aims at creating an avatar system that can transport human presence to a remote location in real time. 

One of the tasks of the competition requires the following task:
>Recipient picks up and hands O/A (Operator/Avatar) the artifact. O/A describes the weight of
the object and places it back on the table.

Thus the human operator must be able to perceive in some ways the weight of an object manipulated by the robotic avatar. Apart from the task of the competition, providing such perception increases the realism and the immersion of the operator, thus improving the overall telexistence experience.


## Objectives 
The objective of this element is to integrate the perception of the weight of objects manipulated by the robotic avatar into our telexistence system.

## Outcomes
The expected outcome is:
- **WeightRetargeting**: an application that reads the weight acting on the robot arms and retargets it towards a human operator by means of haptic feedbacks.

## Milestones
The following milestones have been outlined:

- [Understand how to retarget the weight of an object being manipulated](https://github.com/ami-iit/component_ANA-Avatar-XPRIZE/issues/233)  
  First, a literature review on the topic, with a focus on the ones addressing haptic feedbacks, has to be done. Then we have to understand how we can employ the data collected by our system and the equipment to make the human have a perception of the weight of the object. 
- Develop the weight retargeting application

