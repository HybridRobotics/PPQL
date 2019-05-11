classdef QuadLoadState
	properties
		xL
		vL
		q
		omega
		R
		Omega
		status
	end

	methods
	% constructor as the cable is taut
	function obj = QuadLoadState(xL,vL,q,omega,R,Omega)
		if nargin == 6
			obj.xL = xL;
			obj.vL = vL;
			obj.q = q;
			obj.omega = omega;
			obj.R = R;
			obj.Omega = Omega;
			obj.status = 'taut';
		elseif nargin == 2
			obj.xL = xL;
			obj.vL = vL;
			obj.status = 'slack';
		end
	end

	function [alph,bet] = attitudeToAngles(obj)
		bet = acos(-obj.q(3));
		if bet == 0
			% phi cannot be well defined
			alph = 0;
		elseif obj.q(1) >= 0 && obj.q(2) >= 0
			% phi ranges from 0 to pi/2
			alph = acos(p(1)/sin(bet));
		elseif obj.q(1) >= 0 && obj.q(2) < 0
			% phi ranges from -pi/2 to 0 
			alph = asin(obj.q(2)/sin(bet));
		elseif obj.q(1) < 0 && obj.q(2) >= 0
			% phi ranges from pi/2 to pi
			alph = acos(obj.q(1)/sin(bet));
		else % phi ranges from -pi to -pi/2
			alph = asin(obj.q(2)/sin(bet))-pi;
		end
	end

	function obj = anglesToAttitude(obj,alph,bet)
		obj.q = [-sin(bet)*cos(alph);-sin(bet)*sin(alph);-cos(bet)];
	end
end
end