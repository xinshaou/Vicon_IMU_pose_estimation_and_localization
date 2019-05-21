function U = U_matrix(x, y, z, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z, w_x, w_y, w_z, a_x, a_y, a_z, ng_x, ng_y, ng_z, na_x, na_y, na_z, nbg_x, nbg_y, nbg_z, nba_x, nba_y, nba_z)


U = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; -cos(q_y)/(cos(q_y)^2 + sin(q_y)^2), 0, -sin(q_y)/(cos(q_y)^2 + sin(q_y)^2), 0, 0, 0, 0, 0, 0, 0, 0, 0 ; -(sin(q_x)*sin(q_y))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), -1, (cos(q_y)*sin(q_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0, 0, 0, 0, 0, 0, 0 ; sin(q_y)/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, -cos(q_y)/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, sin(q_x)*sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z), cos(q_x)*cos(q_z), - cos(q_z)*sin(q_y) - cos(q_y)*sin(q_x)*sin(q_z), 0, 0, 0, 0, 0, 0 ; 0, 0, 0, - cos(q_y)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y), -cos(q_x)*cos(q_z), cos(q_y)*cos(q_z)*sin(q_x) - sin(q_y)*sin(q_z), 0, 0, 0, 0, 0, 0 ; 0,  0, 0, cos(q_x)*sin(q_y), -sin(q_x), -cos(q_x)*cos(q_y), 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
 

end
