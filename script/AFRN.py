import numpy as np
import matplotlib.pyplot as plt
import nengo
from nengo.builder import Builder
from nengo.builder import Operator
from nengo.params import NumberParam

class AFRN(nengo.neurons.NeuronType):  # Neuron types must subclass `nengo.Neurons`
    """A rectified linear neuron model."""
    
    # We don't need any additional parameters here;
    # gain and bias are sufficient. But, if we wanted
    # more parameters, we could accept them by creating
    # an __init__ method.
    omega = NumberParam('omega', low=-0.0, low_open=False)
    g = NumberParam('g', low=0, low_open=True)
    thre = NumberParam('thre', low=0, low_open=True)

    def __init__ (self,omega=1.0,g=0.9,thre=0.4):
        super(AFRN, self).__init__()
        self.omega = omega
        self.g = g
        self.thre = thre 
    
    @property
    def _argreprs(self):
        args = []
        if self.omega != 0.0:
            args.append("omega=%s" % self.omega)
        if self.g != 0.9:
            args.append("g=%s" % self.g)
        if self.thre != 0.4:
            args.append("thre=%s" % self.thre)
        return args    
    def gain_bias(self, max_rates, intercepts):
        """Return gain and bias given maximum firing rate and x-intercept."""
        gain = np.ones_like(max_rates)
        bias = np.zeros_like(max_rates)
        self.gain = gain        
        self.bias = bias
        return gain, bias
      
    def rates(self, x, gain, bias):
        """Always use LIFRate to determine rates."""
        J =  x 
        out = np.zeros_like(J)      
        AFRN.step_math(self, dt=0.001, J=J, output=out)
        return out



    def step_math(self, dt, J, output):
        """Implement the LIFRate nonlinearity."""
        
        si_out=[]
        omega = self.omega
        g = self.g
        thre = self.thre             
        x = J
        if x.shape[0] == len(self.gain):
            for i in range(0,len(x)):
                si = np.tanh(g*(x[i]+omega*output[i]))
                if si > thre:
                    si_out.extend([si])
                elif si < thre:
                    si_out.extend([0.])
    
            output[...] = si_out
        else:
            output[...] = x


class SimAFRN(Operator):
    """Set a neuron model output for the given input current.

    Implements ``neurons.step_math(dt, J, output, *states)``.

    Parameters
    ----------
    neurons : NeuronType
        The `.NeuronType`, which defines a ``step_math`` function.
    J : Signal
        The input current.
    output : Signal
        The neuron output signal that will be set.
    states : list, optional (Default: None)
        A list of additional neuron state signals set by ``step_math``.
    tag : str, optional (Default: None)
        A label associated with the operator, for debugging purposes.

    Attributes
    ----------
    J : Signal
        The input current.
    neurons : NeuronType
        The `.NeuronType`, which defines a ``step_math`` function.
    output : Signal
        The neuron output signal that will be set.
    states : list
        A list of additional neuron state signals set by ``step_math``.
    tag : str or None
        A label associated with the operator, for debugging purposes.

    Notes
    -----
    1. sets ``[output] + states``
    2. incs ``[]``
    3. reads ``[J]``
    4. updates ``[]``
    """

    def __init__(self, neurons, J, output, states=None, tag=None):
        super(SimAFRN, self).__init__(tag=tag)
        self.neurons = neurons
        self.J = J
        self.output = output
        self.states = [] if states is None else states

        self.sets = [output]
        self.incs = []
        self.reads = [J]
        self.updates = []

    def _descstr(self):
        return '%s, %s, %s' % (self.neurons, self.J, self.output)

    def make_step(self, signals, dt, rng):
        
        J = signals[self.J]
        output = signals[self.output]

        def step_simafrn():
            self.neurons.step_math(dt, J, output)
        return step_simafrn




@Builder.register(AFRN)
def build_afrn(model, neuron_type, neurons):
    model.operators.append(SimAFRN(
        output=model.sig[neurons]['out'], J=model.sig[neurons]['in'],neurons=neuron_type))
        