-- スクリプトの実行方法：「Arm」かつ「RC6 input > 1800」
--    $ arm throttle
--    $ rc 6 1801
-- スクリプトの実行内容
--    1) Guided modeに変更
--    2) 離陸（高度：5m）
--    3) ROLLの値を更新
--    4) PITCHの値を更新
--    5) YAWの値を更新
--    6) RTL modeで着陸
--    2)~6)の処理中はLOG出力

local takeoff_alt_above_home = 5
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0

local target_roll = 0   -- deg
local target_pitch = 0  -- deg
local target_yaw = 0    -- deg
local climb_rate = 0    -- m/s
local yaw_rad

local stage_start_time_ms = 0
local now_ms
local file_name
local file

function update()

  now_ms = millis()    -- プログラムの実行を開始した時から現在までの時間をミリ秒単位で返す
  --os.date()
  if not arming:is_armed() then     -- Disarmの場合は設定値を初期化する
    stage = 0
    circle_angle = 0
  else                              -- Armの場合
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- RC6の入力値が1800より大きい場合
      if (stage == 0) then          -- Stage0: Guided modeに変更
        if (vehicle:set_mode(copter_guided_mode_num)) then    -- Guided modeへの変更が成功した場合
          yaw_rad = ahrs:get_yaw()
          yaw_cos = math.cos(yaw_rad)
          yaw_sin = math.sin(yaw_rad)
          stage = stage + 1
          file_name = './Result.txt'
          file = io.open(file_name, 'w')
          file:write('now_ms, roll, pitch, yaw\n')
        end
      elseif (stage == 1) then      -- Stage1: 離陸
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then    -- 離陸が成功した場合
          stage = stage + 1
        end

      elseif (stage == 2) then      -- Stage2: 機体が目標の高度に到達したか確認
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "Alt above home: " .. tostring(math.floor(-vec_from_home:z())))
          if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then    -- 機体が目標の高度に到達した場合
            stage = stage + 1
            stage_start_time_ms = now_ms
            target_yaw = yaw_rad
            bottom_left_loc = curr_loc
            gcs:send_text(0, "Roll input stage")
          end
        end

      elseif (stage == 3) then      -- Stage3: ROLLの値を20度に設定
        climb_rate = 0

        if ((now_ms - stage_start_time_ms) < 10000) then        -- ROLLの値を20度に設定（10秒間）
          target_roll = 20
          vehicle:set_target_angle_and_climbrate(target_roll, 0, 0, climb_rate, false, 0)
        elseif ((now_ms - stage_start_time_ms) < 15000) then    -- ROLLの値を0度に設定（5秒間）
          target_roll = 0
          vehicle:set_target_angle_and_climbrate(target_roll, 0, 0, climb_rate, false, 0)
        else
          stage = stage + 1
          stage_start_time_ms = now_ms
          gcs:send_text(0, "Pitch input stage")
        end

      elseif (stage == 4) then      -- Stage4: PITCHの値を20度に設定
        climb_rate = 0

        if ((now_ms - stage_start_time_ms) < 10000) then        -- PITCHの値を20度に設定（10秒間）
          target_pitch = 20
          vehicle:set_target_angle_and_climbrate(0, target_pitch, 0, climb_rate, false, 0)        
        elseif ((now_ms - stage_start_time_ms) < 15000) then    -- PITCHの値を0度に設定（5秒間）
          target_pitch = 0
          vehicle:set_target_angle_and_climbrate(0, target_pitch, 0, climb_rate, false, 0)
        else
          stage = stage + 1
          stage_start_time_ms = now_ms
          gcs:send_text(0, "YAW input stage")
        end

      elseif (stage == 5) then      -- Stage5: Update毎にYAWの値を1度加算
        climb_rate = 0

        if ((now_ms - stage_start_time_ms) < 15000) then    -- Update毎にYAWの値を1度加算（15秒間）
          target_yaw = target_yaw + 1
          if target_yaw >= 360 then
            target_yaw = 0
          end
          vehicle:set_target_angle_and_climbrate(0, 0, target_yaw, climb_rate, false, 0)
        else
          stage = stage + 1
        end

      elseif (stage == 6) then  -- Stage6: RTL modeに変更
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "Finished demo flight, switching to RTL")
        file:close()
      end
    end

    if (stage > 0 and stage < 7) then  -- LOGの出力
      file:write(tostring(now_ms) .. ', ' .. tostring(math.deg(ahrs:get_roll())) .. ', ' .. tostring(math.deg(ahrs:get_pitch())) .. ', ' .. tostring(math.deg(ahrs:get_yaw())) .. '\n')
    end
  end

  return update, 100
end

return update()