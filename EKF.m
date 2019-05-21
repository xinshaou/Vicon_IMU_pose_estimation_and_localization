    function [mean_arr,cov_arr] = EKF(data,vicon,time_vicon,time_data,omg_imu,acc_imu)
    %defining variables for x matrix
%     syms x y z q_x q_y q_z v_x v_y v_z bg_x bg_y bg_z ba_x ba_y ba_z 
% 
%     %defining variables for x dot 
%     syms w_x w_y w_z a_x a_y a_z ng_x ng_y ng_z na_x na_y na_z nbg_x nbg_y nbg_z nba_x nba_y nba_z
% 
%     %defining position, orientation, velocity, gyroscope bias and accelerometer
%     %bias vectors
% 
%     pos = [x ; y ; z];
%     ori = [q_x ; q_y ; q_z];
%     velo = [v_x ; v_y ; v_z];
%     gyro_bias = [bg_x ; bg_y ; bg_z];
%     acc_bias = [ba_x ; ba_y ; ba_z];
% 
%     %defining different variables for R matrix
% 
%     comp_1 = (cos(q_z)*cos(q_y)) - (sin(q_x)*sin(q_z)*sin(q_y));
%     comp_2 = -(cos(q_x)*cos(q_z));
%     comp_3 = (cos(q_z)*sin(q_y)) + (cos(q_y)*sin(q_x)*sin(q_z));
%     comp_4 = (cos(q_y)*sin(q_z)) + (cos(q_z)*sin(q_x)*sin(q_y));
%     comp_5 = cos(q_x)*cos(q_z);
%     comp_6 = (sin(q_z)*sin(q_y)) - (cos(q_z)*cos(q_y)*sin(q_x));
%     comp_7 = -(cos(q_x)*sin(q_y));
%     comp_8 = sin(q_x);
%     comp_9 = cos(q_x)*cos(q_y);
% 
%     %defining Rotation matrix (Z-X-Y)
%     R = [comp_1 comp_2 comp_3 ; comp_4 comp_5 comp_6 ; comp_7 comp_8 comp_9];
% 
% 
%     %defining G matrix
% 
%     G = [cos(q_y) 0 -(cos(q_x)*sin(q_y)) ; 
%         0 1 sin(q_x) ; 
%         sin(q_y) 0 (cos(q_x)*cos(q_y))];
% 
%     %defining angular velocity vector
% 
%     omega = inv(G) * [w_x - bg_x - ng_x;
%                       w_y - bg_y - ng_y;
%                       w_z - bg_z - ng_z];
% 
%     %defining acceleration vector
% 
%     acc = [ 0 ; 0 ; -9.81] + R*[a_x - ba_x - na_x;
%                                 a_y - ba_y - na_y;
%                                 a_z - ba_z - na_z;];
% 
%     %defining angular velocity vector of x, y and z
% 
%     omega_x = omega(1);
%     omega_y = omega(2);
%     omega_z = omega(3);
% 
%     %defining acceleration vector of x, y and z
% 
%     acc_x = acc(1);
%     acc_y = acc(2);
%     acc_z = acc(3);

    %Defining X_dot Vector

    %x_dot = [v_x ; v_y ; v_z ; omega_x ; omega_y ; omega_z ; acc_x ; acc_y ; acc_z ; nbg_x ; nbg_y ; nbg_z ; nba_x ; nba_y ; nba_z];

    %Defining X vector

    %x_vector = [pos ; ori ; velo ; gyro_bias ; acc_bias];

    %Defining Noise vector

    %n = [ng_x ; ng_y ; ng_z ; na_x ; na_y ; na_z ; nbg_x ; nbg_y ; nbg_z ; nba_x ; nba_y ; nba_z];

    %Defining A matrix
    %A = jacobian(x_dot,x_vector);

    %Defining U matrix
    %U = jacobian(x_dot,n);

    %Defining C

     C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ;
          0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];

    %Defining Covariance Matrix

    Q = (1e-3)*eye(12);
    R = (1e-3)*eye(6);
    %d_t = dt * eye(15);
    %d_tq = dt * eye(12);

    %creating a list of all the variables

  %  ran = [x, y, z, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z, w_x, w_y, w_z, a_x, a_y, a_z, ng_x, ng_y, ng_z, na_x, na_y, na_z, nbg_x, nbg_y, nbg_z, nba_x, nba_y, nba_z];
    %size(ran)
    mean = [0 ;0; 0.03; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 
    %disp('mean')
    %size(mean)
    covar = (0.01)*eye(15);

    %dimension = size(time_data,1);
%     omg_imu = [];
%     acc_imu = [];
    
    mean_arr = [];
    cov_arr = [];
%     for i=1:dimension
%         omg_imu(:,i) = data(i).omg;
%         acc_imu(:,i) = data(i).acc;
%     end

    dim_data = size([data.t],2);
    dim_vicon = size(time_vicon,2);
    vicon_ctr = 1;
    imu_data_ctr = 1;

    while true
        if imu_data_ctr > dim_data | vicon_ctr > dim_vicon
            break
        end


        if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
            % match
            disp("at imu counter")
            imu_data_ctr
            if imu_data_ctr == 1
                dt = data(imu_data_ctr).t;      
            else
                dt = data(imu_data_ctr).t - data(imu_data_ctr-1).t;    
            end
            d_t = dt * eye(15);
            d_tq = dt * eye(12);
            %size(ran)
            %disp('in the loop. the size of the subs')
            %size([mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]])
            
            A_cal = A_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %A_cal = subs(A, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);
            U_cal = U_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %U_cal = subs(U, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);
            
            x_dot_cal = x_d_v(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %x_dot_cal = subs(x_dot, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);            
            x_cal = x_v(vicon(1,vicon_ctr), vicon(2,vicon_ctr), vicon(3,vicon_ctr), vicon(4,vicon_ctr), vicon(5,vicon_ctr), vicon(6,vicon_ctr),0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %x_cal = subs(x_vector, ran,[vicon(1:6,vicon_ctr).',[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]]);
            
            %use this for ques 2
            %x_cal = double(subs(x, ran,[[0 0 0 0 0 0], vicon(7:9,i).',[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]]));   
            F = eye(15) + d_t * A_cal;
            Qd = d_tq * Q;
            z = C*x_cal;

            pred_mean = mean + d_t*x_dot_cal;
            pred_covar = F*covar*(F.') + U_cal*Qd*(U_cal.');

            K = pred_covar*(C.')*(inv(C*pred_covar*(C.') + R));
            
%             
            mean = pred_mean + K*(z - (C*pred_mean));
            covar = pred_covar - (K*C*pred_covar);
%             
            
            mean_arr = [mean_arr; mean.'];
            cov_arr = [cov_arr; covar.'];
            
            
            imu_data_ctr = imu_data_ctr + 1;
            vicon_ctr = vicon_ctr + 1;
        else
            % not match
            vicon_ctr = vicon_ctr + 1;
        end

    end

    
    
    
    
    
end