# 차선검출시각화 - 팀명(무제)
<table style="width: 100%; border-collapse: collapse;" align="center"  >
        <tr>
            <td style="width: 50%; text-align: center; vertical-align: middle; border: 1px solid #000;">
                <img src="https://github.com/user-attachments/assets/148b570a-adec-4d39-95b2-1c235b2e1763" alt="이미지(1)" style="max-width: 100%; height: auto;">
            </td>
        </tr>
</table>

## ✨Abstract(작품개요)
해당 작품을 소개하는 개요글을 넣어주세요!

### 💻Architecture(아키텍처)

위 작품에 대한 아키텍처를 그려넣어주세요!

### 🎮Functions(기능)

### 💡Getting Started(시작하기)
ros_ws를 만들고 src폴더에 넣어줄것

### 💡Prerequisites(개발환경)

설치해야할 프로그램 및 빽단에 대한 조치들을 적어주세요!<br>
예를 들어,
- [ROS2 Hubmble](#)
- [WSL: Ubuntu 22.04](#)


### 💡Limit(한계점)
<table style="width: 100%; border-collapse: collapse;" align="center"  >
        <tr>
            <td style="width: 50%; text-align: center; vertical-align: middle; border: 1px solid #000;">
                <img src="https://github.com/user-attachments/assets/4adf09bc-466d-44c6-850b-584697a70d60" alt="이미지(1)" style="max-width: 100%; height: auto;">
            </td>
        </tr>
</table>
현재 내가 연구중인 데이터를 Velodyne LiDAR로 수집한 Rosbag파일이 아닌 Ouster LiDAR로 수집한 Rosbag파일을 사용 중이므로, lane_filter_visualizer에서 기대하는 PointCloud2 필드(ring, noise)가 Ouster LiDAR 데이터에 존재하지 않음이 문제의 원인이다.
Ouster LiDAR는 기본적으로 Velodyne과 다른 포맷의 PointCloud 데이터를 사용하므로, 해당 필드가 없거나 다른 이름으로 저장될 가능성이 크다.
따라서 나중에 Velodyne으로 별도로 데이터를 수집한 후에야 비로소 우리가 원하는 필드데이터를 수집할 수 있기에 그때서야 이 코드를 완전하게 사용할 수 있을 것이다.

### 💡Demo(시연영상)

위 프로그램을 실행했을때의 시연영상을 넣어주세요!


### 📑Contributing(기여 및 업무분장)

누가 어떻게 기여했는지 적어주세요!


## ※Appendix

### I. Q&A(질의응답)

### II. Flow Chart(순서도)

### Referenced Papers and Patents(참고논문 및 특허)
(참고문헌은 `IEEE`스타일로 작성합니다)

  - **Billie Thompson** - *Provided README Template* -
    [PurpleBooth](https://github.com/PurpleBooth)





