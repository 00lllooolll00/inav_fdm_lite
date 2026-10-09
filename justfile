# inav_fdm_lite 快捷命令（需 just + cmake + ninja）
build_dir := "build"

# 默认：构建
default: build

# 配置（Ninja + Debug，方便单步）
configure:
    cmake -S . -B {{build_dir}} -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 构建
build: configure
    cmake --build {{build_dir}}

# 运行自检
test: build
    ./{{build_dir}}/bridge_selftest

# 运行 ctest
ctest: build
    ctest --test-dir {{build_dir}} --output-on-failure

# 启动单实例桥接（参数示例，端口按需改）
run id="1" lat="30.0" lon="120.0" alt="10.0" simport="49000" mav="5762": build
    ./{{build_dir}}/inav_sitl_bridge --id {{id}} --lat {{lat}} --lon {{lon}} --alt {{alt}} \
        -p {{simport}} -l {{simport}} --mav-tcp-port {{mav}} -r 50

# 清理构建目录
clean:
    rm -rf {{build_dir}}
