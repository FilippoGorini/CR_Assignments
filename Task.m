classdef Task < handle

    properties
        xdotbar = [] % reference task velocity
        J = []       % task Jacobian
        A = []       % task internal activation function (the result actually)
    end

    methods (Abstract)
        updateReference(obj)
        updateJacobian(obj)
        updateActivation(obj)
    end
end