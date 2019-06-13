classdef QuadLoadState < DifferentialFlatness
	properties
		xL;
		vL;
		q;
		L;
		omega;
		R;
		Omega;
		status; % 1 (taut) 2(slack) 3(release)
		
		% flat outputs
		aL;
		daL;
		d2aL;
		d3aL;
		d4aL;
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
				obj.status = 1;
			elseif nargin == 2
				obj.xL = xL;
				obj.vL = vL;
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
		function [] = anglesToAttitude(obj,alph,bet)
			obj.q = [-sin(bet)*cos(alph);-sin(bet)*sin(alph);-cos(bet)];
		end
		function flatnessToStateTaut(obj, xL, vL, aL, daL, d2aL, d3aL, d4aL,q,L)
			g = [0;0;-9.8];
			err = 0.01;
			% slack node might appear in the taegment
			if norm(aL - g) >= err
				obj.status = 1; % taut
				[obj.xL,obj.vL,~,obj.q,~,~,obj.R,obj.omega,~,obj.Omega,~,~,~,~,~,~,~,~] = obj.getNomTrajTaut(xL,vL,aL,daL,d2aL,d3aL,d4aL);
				obj.q = q;
				obj.L = L;
			else
				obj.status = 2; % slack, but might be release
				obj.flatnessToStateSlack(xL,vL,aL,q,L);
			end
		end
		function flatnessToStateSlack(obj, xL, vL, aL, q, L)
			obj.xL = xL;
			obj.vL = vL;
			obj.aL = aL;
			obj.q = q;
			obj.L = L;
			% other undefined states
			obj.omega = zeros(3,1);
			obj.R = eye(3);
			obj.Omega = zeros(3,1);
		end
		function visualize(obj)
			if obj.status == 1
				obj.visualize_taut();
			elseif obj.status == 2
				obj.visualize_slack();
			elseif obj.status == 3
				obj.visualize_release();
			end
		end		
		function visualize_taut(obj)
			s.L = 0.175; %length of quadrotor boom
			s.R = 0.1; %radius of propeller prop

			% Extract state x
			xL = obj.xL;
			p = obj.q;
			R = obj.R;
			L = obj.L;
			xQ = xL - L*p;

			%     plot3(xQ(1), xQ(2), xQ(3), 'r.') ;

			BRW = R' ;

			point1 = BRW'*[s.L,0,0]';
			point2 = BRW'*[0,s.L,0]';
			point3 = BRW'*[-s.L,0,0]';
			point4 = BRW'*[0,-s.L,0]';

			nprop = 40;
			propangs = linspace(0,2*pi,nprop);
			proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

			wp = xQ ;
			wp1 = wp + point1;
			wp2 = wp + point2;
			wp3 = wp + point3;
			wp4 = wp + point4;
			wp_cable_attach = wp + BRW'*[0;0;0] ; %[0;0;-params.cable_attach_d];

			prop1 = proppts + wp1*ones(1,nprop);
			prop2 = proppts + wp2*ones(1,nprop);
			prop3 = proppts + wp3*ones(1,nprop);
			prop4 = proppts + wp4*ones(1,nprop);

			lwp = 2 ;
			lw1 = 1 ;
			lwc = 2 ;
			lwl = 2 ;

			s.qhandle1 = line([wp1(1),wp3(1)],[wp1(2),wp3(2)],[wp1(3),wp3(3)]); hold on ;
			s.qhandle2 = line([wp2(1),wp4(1)],[wp2(2),wp4(2)],[wp2(3),wp4(3)]);
			set(s.qhandle1,'Color','k', 'LineWidth',lw1);
			set(s.qhandle2,'Color','k', 'LineWidth',lw1);

			s.hprop1 = plot3(prop1(1,:),prop1(2,:),prop1(3,:),'r-', 'LineWidth',lwp);
			s.hprop2 = plot3(prop2(1,:),prop2(2,:),prop2(3,:),'b-', 'LineWidth',lwp);
			s.hprop3 = plot3(prop3(1,:),prop3(2,:),prop3(3,:),'b-', 'LineWidth',lwp);
			s.hprop4 = plot3(prop4(1,:),prop4(2,:),prop4(3,:),'b-', 'LineWidth',lwp);

			s.hload = plot3(xL(1,1), xL(2,1), xL(3,1), 'ko', 'LineWidth',lwl) ;
			s.cable = plot3([wp_cable_attach(1) xL(1,1)], [wp_cable_attach(2) xL(2,1)], [wp_cable_attach(3) xL(3,1)], 'k') ;
			s.cable_attach = plot3([wp(1) wp_cable_attach(1)], [wp(2) wp_cable_attach(2)], [wp(3) wp_cable_attach(3)], 'r') ;
		end
		function visualize_slack(obj)
			s.L = 0.175; %length of quadrotor boom
			s.R = 0.1; %radius of propeller prop

			% Extract state x
			xL = obj.xL;
			p = obj.q;
			R = obj.R;
			L = obj.L;
			xQ = xL - L*p;

			%     plot3(xQ(1), xQ(2), xQ(3), 'r.') ;

			BRW = R' ;

			point1 = BRW'*[s.L,0,0]';
			point2 = BRW'*[0,s.L,0]';
			point3 = BRW'*[-s.L,0,0]';
			point4 = BRW'*[0,-s.L,0]';

			nprop = 40;
			propangs = linspace(0,2*pi,nprop);
			proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

			wp = xQ ;
			wp1 = wp + point1;
			wp2 = wp + point2;
			wp3 = wp + point3;
			wp4 = wp + point4;
			wp_cable_attach = wp + BRW'*[0;0;0] ; %[0;0;-params.cable_attach_d];

			prop1 = proppts + wp1*ones(1,nprop);
			prop2 = proppts + wp2*ones(1,nprop);
			prop3 = proppts + wp3*ones(1,nprop);
			prop4 = proppts + wp4*ones(1,nprop);

			lwp = 2 ;
			lw1 = 1 ;
			lwc = 2 ;
			lwl = 2 ;

			s.qhandle1 = line([wp1(1),wp3(1)],[wp1(2),wp3(2)],[wp1(3),wp3(3)]); hold on ;
			s.qhandle2 = line([wp2(1),wp4(1)],[wp2(2),wp4(2)],[wp2(3),wp4(3)]);
			set(s.qhandle1,'Color','k', 'LineWidth',lw1);
			set(s.qhandle2,'Color','k', 'LineWidth',lw1);

			s.hprop1 = plot3(prop1(1,:),prop1(2,:),prop1(3,:),'r-', 'LineWidth',lwp);
			s.hprop2 = plot3(prop2(1,:),prop2(2,:),prop2(3,:),'b-', 'LineWidth',lwp);
			s.hprop3 = plot3(prop3(1,:),prop3(2,:),prop3(3,:),'b-', 'LineWidth',lwp);
			s.hprop4 = plot3(prop4(1,:),prop4(2,:),prop4(3,:),'b-', 'LineWidth',lwp);

			s.hload = plot3(xL(1,1), xL(2,1), xL(3,1), 'ko', 'LineWidth',lwl) ;
			s.cable = plot3([wp_cable_attach(1) xL(1,1)], [wp_cable_attach(2) xL(2,1)], [wp_cable_attach(3) xL(3,1)], 'k-') ;
			s.cable_attach = plot3([wp(1) wp_cable_attach(1)], [wp(2) wp_cable_attach(2)], [wp(3) wp_cable_attach(3)], 'r') ;
		end
		function visualize_release(obj)
			% Extract state x
			xL = obj.xL;
			lwl = 2;
			s.hload = plot3(xL(1), xL(2), xL(3), 'ko', 'LineWidth',lwl) ;
		end
	end
	
end