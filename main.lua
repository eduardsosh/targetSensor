

local function rotation_matrix(pitch, yaw, roll)
    local cos = math.cos
    local sin = math.sin

    -- Rotation matrix for pitch (around x-axis)
    local R_x = {
        {1, 0, 0},
        {0, cos(pitch), -sin(pitch)},
        {0, sin(pitch), cos(pitch)}
    }

    -- Rotation matrix for yaw (around z-axis)
    local R_y = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    }

    -- Rotation matrix for roll (around y-axis)
    local R_z = {
        {cos(roll), 0, sin(roll)},
        {0, 1, 0},
        {-sin(roll), 0, cos(roll)}
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

    local R_yz = mat_mult(R_y, R_z)
    local R = mat_mult(R_yz, R_x)
    return R
end

local function calculate_new_point(aircraft_coords, pitch, yaw, roll, distance, vertical_offset, horizontal_offset)
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

	print("x: ", string.format("%.2f",P_local[1]),"\ny: ", string.format("%.2f",P_local[2]),"\nz: ", string.format("%.2f",P_local[3]))
	print("\n")


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
    local horizontal_offset = math.atan2(P_local[2], P_local[1])

    return vertical_offset, horizontal_offset
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

--[[
-- Example usage
compass = 0
local aircraft_coords = {0, 0, 10} -- Aircraft coordinates (x, y, z)
local pitch = math.rad(0) -- Pitch in radians
--local yaw = math.rad() -- Yaw (heading) in radians
local yaw = ((1-compass-0.25)%1)*360
yaw = math.rad(yaw)
print(yaw)
local roll = math.rad(0) -- Roll in radians
local distance = 10 -- Distance to the ground point in meters
local vertical_offset = math.rad(0) -- Vertical offset from the nose in radians
local horizontal_offset = math.rad(0) -- Horizontal offset from the nose in radians

--local ground_point = calculate_new_point(aircraft_coords, pitch, yaw, roll, distance, vertical_offset, horizontal_offset)
---print("x: ", string.format("%.2f",ground_point[1]),"\ny: ", string.format("%.2f",ground_point[2]),"\nz: ", string.format("%.2f",ground_point[3]))
--]]

-- Example usage
local aircraft_coords = {0, 0, 10} -- Aircraft coordinates (x, y, z)
local pitch = math.rad(-45) -- Pitch in radians
local compass = -0.25
local yaw = ((1 - compass - 0.25) % 1) * 360
yaw = math.rad(yaw)
local roll = math.rad(0) -- Roll in radians
local target_coords = {10, 0, 0} -- Target coordinates (x, y, z)

local vertical_offset, horizontal_offset = calculate_offsets(aircraft_coords, target_coords, roll, yaw, pitch)
print("Vertical offset (deg):", math.deg(vertical_offset))
print("Horizontal offset (deg):", math.deg(horizontal_offset))


