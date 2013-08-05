function [rate_of_increase,final_kp,best_kp] = analyze_oscillation(ITERATIONS, kp)
%'Test1_y_rotation.txt'

%This function analyzes the change of angle phi over time

rate_of_increase = zeros(ITERATIONS,2);


% Run the QuadSim program, with the starting Kp and increase it. 
%Analyze the resulting angle to find its local extrema
for i= 1:ITERATIONS
	phi_t = QuadSim('Test1_y_rotation.txt',kp);

	%Make all elements positive. This means minima -> maxima.
	for m = 1:length(phi_t)
		if phi_t(m) < 0
			phi_t(m) = -phi_t(m);
		end
	end
	
	[peaks] = findpeaks(phi_t);
	
	counter = 0;
	for x = 1:length(peaks)
		
		if x ~= length(peaks)
			rate_of_increase(i,1) = rate_of_increase(i,1) + (peaks(x+1)/peaks(x));
			counter = counter + 1;
		end
		
	end
	if counter > 0
		rate_of_increase(i,1) = rate_of_increase(i,1) / counter;
	end
	
	rate_of_increase(i,2) = kp;
	
	kp = kp + 0.1;

end
final_kp = kp;
roi = rate_of_increase(1:length(rate_of_increase),1);
foomax = min(roi);
best_kp = 0;

for i = 1:length(roi)
	if rate_of_increase(i,1) == foomax;
		best_kp = rate_of_increase(i,2);
		break;
	end
end
