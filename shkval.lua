rot_x = 0
rot_y = 0
pivot_x = 0
pivot_y = 0
gps_x = 0
gps_y = 0
alt = 0
laser_d = 0
lock = 0
pitch = 0
roll = 0

my_x = 0
my_y = 0
my_z = 0 --altitude
locked = false
p_lock = false
pi2 = math.pi * 2

function onTick()
	-- camera pivot commands
	pivot_x = input.getNumber(1)
	pivot_y = input.getNumber(2)

	gps_x = input.getNumber(3)
	gps_y = input.getNumber(4)
	alt = input.getNumber(5)

	aircraft_coords = {gps_x,gps_y,alt}

	laser_d = input.getNumber(6)

	-- convert from turns to radians
	pitch = input.getNumber(7) * pi2
	roll = input.getNumber(8) * pi2
	compass = input.getNumber(9)
	yaw = ((1-compass-0.25)%1)*360
	yaw = math.rad(yaw)

	lock = input.getBool(1)

	-- camera rotation output
	-- currently this is non--lock mode

	rot_x = rot_x + pivot_x*0.01
	rot_y = rot_y + pivot_y*0.01

	rot_x = clamp(rot_x,-1,1)
	rot_y = clamp(rot_y,-1,1)

	--get rot x and rot y in radians
	--rot_? -> turns -> radians
	horizontal_offset = (rot_x/8)*pi2
	vertical_offset = (rot_y/8)*pi2

	-- aircraft_coords, pitch, yaw, roll, distance, vertical_offset, horizontal_offset
	if(lock and not p_lock)then
		if(locked)then
			locked = false
		else
			locked = true
		end
	end


	if(lock and not p_lock)then
		target_coords = calculate_new_point(aircraft_coords,pitch,yaw,roll,laser_d,vertical_offset,horizontal_offset)
		target_x = target_coords[1]
		target_y = target_coords[2]
		target_z = target_coords[3]
	end




	output.setNumber(3,target_x)
	output.setNumber(4,target_y)
	output.setNumber(5,target_z)


	if(locked)then
		--aircraft_coords, target_coords, pitch, yaw, roll
		-- pitch might be inverted
		offsets = calculate_offsets(aircraft_coords,target_coords,roll,yaw,pitch)

		--offsets are in radians, convert to range -1 to 1
		--offset(rad) -> turns -> range
		rot_x = (offsets[2]/pi2)*8
		rot_y = (offsets[1]/pi2)*8
	end


	output.setNumber(1, rot_x)
	output.setNumber(2, rot_y)

	p_lock = lock
end

function onDraw()
	if(locked)then
		screen.setColor(255,0,0)
		screen.drawRect(2,2,5,5)
	end
end


function rotation_matrix(pitch, yaw, roll)
    local cos = math.cos
    local sin = math.sin

    -- Rotation matrix for pitch (around x-axis)
    local R_x = {
        {1, 0, 0},
        {0, -cos(roll), sin(roll)},
        {0, sin(roll), cos(roll)}
    }

    -- Rotation matrix for yaw (around z-axis)
    local R_y = {
        {cos(yaw), -sin(yaw), 0},
        {-sin(yaw), -cos(yaw), 0},
        {0, 0, 1}
    }

    -- Rotation matrix for roll (around y-axis)
    local R_z = {
        {cos(pitch), 0, sin(pitch)},
        {0, -1, 0},
        {-sin(pitch), 0, cos(pitch)}
    }

    -- Combine rotations R = R_y * R_z * R_x
    local function mat_mult(A, B)
        local C = {}
        for i = 1, 3 do
            C[i] = {}
            for j = 1, 3 do
                C[i][j] = 0
                for k = 1, 3 do
                    C[i][j] = C[i][j] + A[i][k] * B[k][j]
                end
            end
        end
        return C
    end

    local R_yx = mat_mult(R_y, R_x)
    local R = mat_mult(R_yx, R_z)
    return R
end

function calculate_new_point(aircraft_coords, pitch, yaw, roll, distance, vertical_offset, horizontal_offset)
    local x0, y0, z0 = aircraft_coords[1], aircraft_coords[2], aircraft_coords[3]
    local cos = math.cos
    local sin = math.sin

    -- Get the rotation matrix
    local R = rotation_matrix(pitch, yaw, roll)

    -- Local coordinates of the offset point
    local P_local = {
        distance * cos(vertical_offset) * cos(horizontal_offset),
        distance * cos(vertical_offset) * sin(horizontal_offset),
        -distance * sin(vertical_offset)  -- Negative because vertical offset to ground
    }


    -- Transform to global coordinates
	--R @ P_local
    local P_global = {
        R[1][1] * P_local[1] + R[1][2] * P_local[2] + R[1][3] * P_local[3],
        R[2][1] * P_local[1] + R[2][2] * P_local[2] + R[2][3] * P_local[3],
        R[3][1] * P_local[1] + R[3][2] * P_local[2] + R[3][3] * P_local[3]
    }

    -- Calculate final coordinates
    local x = x0 + P_global[1]
    local y = y0 + P_global[2]
    local z = z0 + P_global[3]

    return {x, y, z}
end

function calculate_offsets(aircraft_coords, target_coords, pitch, yaw, roll)
    local x0, y0, z0 = aircraft_coords[1], aircraft_coords[2], aircraft_coords[3]
    local xt, yt, zt = target_coords[1], target_coords[2], target_coords[3]

    -- Get the rotation matrix
    local R = rotation_matrix(pitch, yaw, roll)
    local R_inv = transpose_matrix(R) -- Inverse of the rotation matrix

    -- Transform the global coordinates to the local coordinates
	-- might need to invert y again
    local P_global = {
        xt - x0,
        yt - y0,
        zt - z0
    }

    local P_local = {
        R_inv[1][1] * P_global[1] + R_inv[1][2] * P_global[2] + R_inv[1][3] * P_global[3],
        R_inv[2][1] * P_global[1] + R_inv[2][2] * P_global[2] + R_inv[2][3] * P_global[3],
        R_inv[3][1] * P_global[1] + R_inv[3][2] * P_global[2] + R_inv[3][3] * P_global[3]
    }

    -- Calculate the distance
    local distance = math.sqrt(P_local[1]^2 + P_local[2]^2 + P_local[3]^2)

    -- Calculate vertical and horizontal offsets
    local vertical_offset = -math.asin(P_local[3] / distance) -- Negative because we used -sin in the original code
    local horizontal_offset = atan2(P_local[2], P_local[1])

    return {vertical_offset, horizontal_offset}
end

function transpose_matrix(M)
    local T = {}
    for i = 1, 3 do
        T[i] = {}
        for j = 1, 3 do
            T[i][j] = M[j][i]
        end
    end
    return T
end

function clamp(x,small,high)
	return math.min(math.max(small,x),high)
end

function atan2(y, x)
    if x > 0 then
        return math.atan(y / x)
    elseif x < 0 and y >= 0 then
        return math.atan(y / x) + math.pi
    elseif x < 0 and y < 0 then
        return math.atan(y / x) - math.pi
    elseif x == 0 and y > 0 then
        return math.pi / 2
    elseif x == 0 and y < 0 then
        return -math.pi / 2
    else
        return 0 -- x == 0 and y == 0
    end
end



