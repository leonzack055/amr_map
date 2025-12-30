-- 直线特征提取配置文件
-- 用于从Cartographer生成的地图中提取直线特征

-- 主配置
line_extraction = {
  enable_line_extraction = true,
  enable_adaptive_params = true,
  enable_debug_output = false,
  
  -- 预处理配置
  preprocessing = {
    -- 调试选项
    enable_debug_output = false,
    
    -- 二值化
    use_otsu_threshold = true,
    manual_threshold = 50.0,
    adaptive_block_size = 15,
    adaptive_c = 2.0,
    
    -- 降噪
    enable_median_filter = true,
    enable_gaussian_filter = false,
    filter_kernel_size = 3,
    gaussian_sigma = 1.0,
    
    -- 形态学操作
    enable_morphology = true,
    enable_opening = true,
    enable_closing = true,
    morphology_kernel_size = 3,
    morphology_kernel_shape = "rectangle",
    
    -- 边缘检测
    enable_canny = true,
    canny_low_threshold = 50.0,
    canny_high_threshold = 150.0,
    canny_kernel_size = 3,
    use_l2_gradient = true
  },
  
  -- 霍夫变换配置
  hough_transform = {
    enabled = true,
    
    -- 基本参数
    rho_resolution = 1.0,
    theta_resolution = math.pi / 180.0,
    hough_vote_threshold = 50,
    
    -- 线段参数
    min_line_length_pixels = 30.0,
    max_line_gap_pixels = 10.0,
    min_line_length_meters = 0.5,
    max_line_gap_meters = 0.2,
    max_num_lines = 100,
    
    -- 过滤参数
    confidence_threshold = 0.7,
    min_points_for_line = 10,
    
    -- 融合参数
    max_angle_diff = 5.0 * math.pi / 180.0,
    max_distance_diff = 0.1,
    min_overlap_ratio = 0.3
  },
  
  -- LSD配置
  lsd = {
    enabled = true,
    
    -- 基本参数
    scale = 0.8,
    sigma_scale = 0.6,
    quant = 2.0,
    ang_th = 22.5,
    log_eps = 0.0,
    density_th = 0.7,
    n_bins = 1024,
    
    -- 过滤参数
    min_line_length_meters = 0.3,
    confidence_threshold = 0.6,
    min_support_points = 5
  },
  
  -- 后处理配置
  post_processing = {
    -- 去重配置
    enable_deduplication = true,
    duplicate_angle_threshold = 3.0 * math.pi / 180.0,
    duplicate_distance_threshold = 0.05,
    duplicate_overlap_threshold = 0.8,
    
    -- 连接配置
    enable_connection = true,
    connection_angle_threshold = 5.0 * math.pi / 180.0,
    connection_gap_threshold = 0.15,
    
    -- 分类配置
    enable_classification = true,
    wall_length_threshold = 1.0,
    corridor_width_threshold = 2.0,
    obstacle_length_threshold = 0.5
  },
  
  -- 输出配置
  output = {
    output_json = true,
    output_csv = true,
    output_yaml = true,
    output_visualization = true,
    output_protobuf = true,
    output_statistics = true,
    
    output_directory = "./line_features/",
    base_filename = "map_lines",
    separate_by_type = true,
    include_debug_images = false
  },
  
  -- 性能配置
  performance = {
    enable_parallel_processing = true,
    num_threads = 0,  -- 0表示使用所有可用线程
    max_image_width = 2048,
    max_image_height = 2048,
    enable_block_processing = true,
    block_size = 512,
    enable_memory_optimization = true
  }
}

-- 针对不同地图类型的预设配置
presets = {
  -- 室内环境
  indoor = {
    preprocessing = {
      use_otsu_threshold = true,
      canny_low_threshold = 30.0,
      canny_high_threshold = 100.0,
      enable_morphology = true,
      morphology_kernel_size = 3
    },
    hough_transform = {
      hough_vote_threshold = 40,
      min_line_length_meters = 0.5,
      confidence_threshold = 0.7
    },
    lsd = {
      density_th = 0.7,
      min_line_length_meters = 0.3,
      confidence_threshold = 0.6
    }
  },
  
  -- 室外环境
  outdoor = {
    preprocessing = {
      use_otsu_threshold = true,
      canny_low_threshold = 50.0,
      canny_high_threshold = 150.0,
      enable_morphology = true,
      morphology_kernel_size = 5
    },
    hough_transform = {
      hough_vote_threshold = 30,
      min_line_length_meters = 1.0,
      confidence_threshold = 0.5
    },
    lsd = {
      density_th = 0.5,
      min_line_length_meters = 0.5,
      confidence_threshold = 0.5
    }
  },
  
  -- 高精度模式
  high_precision = {
    preprocessing = {
      canny_low_threshold = 20.0,
      canny_high_threshold = 80.0,
      filter_kernel_size = 5
    },
    hough_transform = {
      hough_vote_threshold = 60,
      min_line_length_meters = 0.3,
      confidence_threshold = 0.8
    },
    lsd = {
      density_th = 0.8,
      min_line_length_meters = 0.2,
      confidence_threshold = 0.7
    }
  },
  
  -- 快速模式
  fast = {
    preprocessing = {
      canny_low_threshold = 80.0,
      canny_high_threshold = 200.0,
      enable_morphology = false
    },
    hough_transform = {
      hough_vote_threshold = 20,
      min_line_length_meters = 1.0,
      confidence_threshold = 0.4
    },
    lsd = {
      enabled = false  -- 在快速模式下禁用LSD
    }
  }
}

-- 函数：应用预设配置
function apply_preset(preset_name)
  if presets[preset_name] then
    for category, settings in pairs(presets[preset_name]) do
      for key, value in pairs(settings) do
        line_extraction[category][key] = value
      end
    end
    print("应用预设配置: " .. preset_name)
  else
    print("未找到预设配置: " .. preset_name)
  end
end

-- 函数：打印当前配置
function print_config()
  print("当前直线特征提取配置:")
  print("  启用直线提取: " .. tostring(line_extraction.enable_line_extraction))
  print("  启用自适应参数: " .. tostring(line_extraction.enable_adaptive_params))
  print("  霍夫变换: " .. tostring(line_extraction.hough_transform.enabled))
  print("  LSD检测: " .. tostring(line_extraction.lsd.enabled))
  print("  输出JSON: " .. tostring(line_extraction.output.output_json))
  print("  输出CSV: " .. tostring(line_extraction.output.output_csv))
end

-- 示例使用
-- apply_preset("indoor")
-- print_config()