classdef QuadLoadDifferentialState < QuadLoadState
	properties
		% flat outputs used for taut case
		daL;
		d2aL;
		d3aL;
		d4aL;
		% flat outputs used for slack case
		xQ;
	end
	methods
		function flatOutputsToStateConnect(obj, t, xL, vL, aL, daL, d2aL, d3aL, d4aL, q, L)
			g = [0;0;-9.8];
			err = 0.01;
			% check whether the cable is slack or taut
			if norm(aL - g) >= err
				obj.flatOutputsToStateTaut(t, xL, vL, aL, daL, d2aL, d3aL, d4aL, q, L)
			else
				obj.flatOutputsToStateSlack(t, xL, vL, aL, q, L);
			end
		end

		function flatOutputsToStateTaut(obj, t, xL, vL, aL, daL, d2aL, d3aL, d4aL, q, L)
			[xL, vL, ~, q, ~, ~, R, omega, ~, Omega, ~, ~, ~, ~, ~, ~, ~, ~] = obj.getNomTrajTaut(xL, vL, aL, daL, d2aL, d3aL, d4aL);
			obj.status = 1;
			obj.time = t;
			obj.xL = xL;
			obj.vL = vL;
			obj.aL = aL;
			obj.q = q;
			obj.L = L;
			obj.omega = omega;
			obj.R = R;
			obj.Omega = Omega;
			% flat outputs
			obj.daL = daL;
			obj.d2aL = d2aL;
			obj.d3aL = d3aL;
			obj.d4aL = d4aL;
			obj.xQ = NaN(3,1);
		end

		function flatOutputsToStateSlack(obj, t, xL, vL, aL, q, L)
			obj.status = 2;
			obj.time = t;
			obj.xL = xL;
			obj.vL = vL;
			obj.aL = aL;
			obj.q = q;
			obj.L = L;
			% other states
			obj.omega = zeros(3,1);
			obj.R = eye(3);
			obj.Omega = zeros(3,1);
			% flat outputs undefined since useless
			obj.daL = NaN(3,1);
			obj.d2aL = NaN(3,1);
			obj.d3aL = NaN(3,1);
			obj.d4aL = NaN(3,1);
			obj.xQ = xL - q*L;
		end

		function flatOutputsToStateRelease(obj, t, xL, vL, aL, q, L)
			obj.status = 3;
			obj.time = t;
			obj.xL = xL;
			obj.vL = vL;
			obj.aL = aL;
			obj.q = q;
			obj.L = L;
			% other states
			obj.omega = zeros(3,1);
			obj.R = eye(3);
			obj.Omega = zeros(3,1);
			% flat outputs undefined since useless
			obj.daL = NaN(3,1);
			obj.d2aL = NaN(3,1);
			obj.d3aL = NaN(3,1);
			obj.d4aL = NaN(3,1);
			obj.xQ = NaN(3,1);
		end
	end
end