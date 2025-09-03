import dagger
from dagger import dag, function, object_type

CONTAINER_BUILD_ENV_CMD = [
    "bash", "-c",
    """
    apt-get update &&
    apt-get install -y libgmock-dev python3-pip clang-format cppcheck lcov &&
    pip3 install cython cpplint pytest pytest-cov &&
    mkdir -p /ros2_ws/src &&
    mkdir -p /ros2_ws/result
    """
]

CONTAINER_BUILD_CMD = [
    "bash", "-c",
    """
    rm -rf build install log &&
    source /opt/ros/humble/setup.bash &&
    colcon build 2>&1 | tee result/build.log &&
    python3 /app/scripts/buildlog_simple_junit.py result/build.log result/build.xml || true
    """
]

CONTAINER_TEST_CMD = [
    "bash", "-c",
    """
    source /opt/ros/humble/setup.bash &&
    colcon test &&
    colcon test-result || true
    """
]

@object_type
class Basics:
    @function
    def build_env(self, source: dagger.Directory) -> dagger.Container:
        """ビルド環境を用意します"""
        mod_src = dag.current_module().source() 
        return (
            dag.container()
            .from_("osrf/ros:humble-desktop")
            .with_directory("/app", mod_src)
            .with_mounted_directory("/ros2_ws/src", source)
            .with_workdir("/ros2_ws")
            .with_exec(CONTAINER_BUILD_ENV_CMD)
        )

    @function
    def build(self, source: dagger.Directory) -> dagger.Directory:
        """ビルド環境でビルドします"""
        return (
            self.build_env(source=source)
            .with_exec(CONTAINER_BUILD_CMD)
            .directory("result")
        )

    @function
    def test(self, source: dagger.Directory) -> dagger.Directory:
        """ビルド済み環境で単体テストします"""
        return (
            self.build_env(source=source)
            .with_exec(CONTAINER_BUILD_CMD)
            .with_exec(CONTAINER_TEST_CMD)
            .directory("build")
        )

    @function
    def artifacts(self, source: dagger.Directory) -> dagger.Directory:
        """ビルド済み環境のフォルダを返します"""
        return (
            self.build_env(source=source)
            .with_exec(CONTAINER_BUILD_CMD)
            .directory("install")
        )
