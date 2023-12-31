cmake_minimum_required(VERSION 3.5)
project(add_two_values)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# ここまでは自動生成


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)  # CMakeLists.txtがあるディレクトリからの相対パスでインクルードファイルへのパスを指定

add_executable(add_two_values_exe  # add_two_values_exeはCMakeLists.txt内で使う変数みたいなもの
  src/main.cpp
  src/node.cpp  # これらをコンパイルしてできた実行ファイルにadd_two_values_exeと名付けた
)

ament_target_dependencies(add_two_values_exe
  rclcpp
  std_msgs
) # add_two_values_exeが依存しているパッケージを明示（find_packageで取ってきたもの）


# ビルドによって作成されたファイルの置き場所を指定する場合はinstall()を使う
install(
  TARGETS add_two_values_exe  # この実行ファイルをどこに置くか次の行で指定
  DESTINATION lib/${PROJECT_NAME}  # ビルドを実行したディレクトリに作成されるinstallディレクトリ下にこのようにディレクトリを作成してそこに置かれる
                                   # PROJECT_NAMEはproject(add_two_values)で指定した名前
)
install(
  DIRECTORY launch  # このディレクトリをどこに置くか次の行で指定（ローンチファイルをここに置かないとsourceしてもパスが通らないので面倒）
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package() # ここも自動生成。脳死で書くもの。
