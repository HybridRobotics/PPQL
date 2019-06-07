classdef SystemParameters < handle
	properties
		% System parameters
		mQ = 0.500;
		mL = 0.087;
		J = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3];
		g = 9.81;
		e1 = [1;0;0];
		e2 = [0;1;0];
		e3 = [0;0;1];
		L = 0.5;
	end
end