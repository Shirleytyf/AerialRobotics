function path = simplify_path(map, path)
    if size(path,1) == 0
        error('The path is empty!');
    end
    % 创建一个名为“keep_pBooL”的向量，其长度与输入路径中的点数相同。
    % 将所有值初始化为“false”，表示不应在简化路径中保留任何点。
    keep_pBooL = false(size(path,1),1);
    % 输入路径的第一个点和最后一个点设为true.
    keep_pBooL(1) = 1;     
    keep_pBooL(end) = 1;
    last_keep = path(1,:);

    for i = 2:size(path,1)-1
        % ~check_line_collision() == 1 : No collision
	    %  check_line_collision() == 1 : collision
        if (~check_line_collision(last_keep, path(i,:)) && check_line_collision(last_keep, path(i+1,:))) 
            last_keep = path(i,:);
            keep_pBooL(i) = 1;
        end
    end
    path = path(keep_pBooL,:);
    
    function c = check_line_collision(start_pos, end_pos)
        
        pts = map.generateiIntermediaPoints(start_pos, end_pos);
        
        % Collision detection
        c = map.collide(pts);
        c = any(c);
    end
end
