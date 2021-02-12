# NB-using-CMSIS-DSP

Implementation of Naive_bayes Classifier using CMSIS-DSP on stm32f4 mcu.

## Description

The repo contains python code for training the Guassian Naive Bayes Classifier using some dummy data given by tutorial https://www.datacamp.com/community/tutorials/naive-bayes-scikit-learn using scikit-learn package. After training is done, various parameters like Gaussian averages, Variances are extracted from it.

Then the same parameters are used in CMSIS DSP naive bayes algorithm and predictions are computed on Stm32f4 mcu.

## Program Logic

The Basic job of classifier is to take some inputs and classify them into given outputs. So here the inputs are called features. The inputs or features to naive bayes algorithm are two viz Weather and Temperature, the naive bayes classifier calculates the probability using a math equation determining whether we can play outside given these two inputs.So the output of the classifier can be "Yes" or "No". In other words we can say that there are two options for the classifier to choose from.

After we train the naive bayes algorithm for a set of inputs and therefore we get the parameters like Gaussian averages, Variances we load the parameters into the arm CMSIS_dsp naive bayes function and compute the probabilty given the inputs whether using a program or from sensors.

In case processing of inputs using microcontroller we can either code the naive bayes function by ourselves or we have to use the CMSIS-DSP provided function . In CMSIS-DSP the same function has 3 variants depending on the processor type like Neon, Helium which use architecture specific instructions to do fast calculation.
The naive bayes function calculates probability for the given inputs and then i have implemented sleep function to put the processor in sleep mode for 10ms Curent(12.2mA), 1sec current(6.6mA) and i have used timer interrupt to wakeup the processor from sleep mode and again calculate the probability. After the probability is calculated i used uart to display it on console.

I have used the below Oscillator Configuration for Time Period Calculation:
* APB1 TIM3   = 50 Mhz  (internal OSC PLL used)
* Prescalar Value(PRE) = 4999
* Period value (ARR)  =  20000  //   20000 = 2sec , 10000 = 1sec,  1ms = 10000/1000 = 10


