\section*{0 引言}

随着我国综合国力的不断提升，国家对于道路建设的投人越来越多，公路运输已成为物资输送的主要方式之一。其中，牵引挂车作为一种机动灵活、低成本的运输设备，常用于运输作业 ${ }^{[1]}$ 。泊车是车辆行驶过程中不可避免的一环，也是令驾驶者＂头疼＂的环节，因为几乎所有的车辆都存在盲区，若驾驶者经验不足就极易导致碰撞事故。由于挂车的车厢较长，使得牵引挂车的盲区范围更广，驾驶者甚至难以通过左右后视镜来观测后方情况 ${ }^{[2]}$ ，这使人工泊车变得尤为困难。此外，泊车过程中若对方向盘的调整不适宜，还会造成牵引挂车的折叠 ${ }^{[3]}$ ，即使是经验丰富的驾驶员也很难将车辆一次性成功泊人车位。因此，为牵引挂车设计一套成熟的自动泊车系统尤为重要。

自动泊车系统主要包含 3 个环节：环境感知、路径规划与路径跟踪 ${ }^{[4]}$ 。环境感知主要是车辆利用摄像头等传感器获得周围环境信息以及车辆当前的位姿（位置与航向角）信息。路径规划是根据车辆当前位置，在具有障碍物的环境中按一定的评价标准寻找一条到达目标点的无碰撞路径 ${ }^{[5]}$ 。路径跟踪阶段通常是将车辆当前的实际位姿信息与参考位姿信息的偏差作为控制器输人，得到相应输出：车速或前轮转角等，从而控制车辆做出相应调整。牵引挂车相当于两个刚体通过较接点的柔性连接，控制难度相对较大 ${ }^{[6]}$ ，目前大多数研究都是关于其前向运动展开的，如文［7－9］等。在泊车方面，文［10］采用混合 A＊与 Reeds－Sheep 曲线结合的方法进行泊车路径规划，并利用预瞄控制进行路径跟踪。但预瞄控制在跟踪大曲率路径时，跟踪精度会受预瞄距离的影响，导致其对停车位的尺寸要求较大。文［11］以铰接角（牵引车与挂车的航向角之差）为切入点，研究了牵引挂车直线倒车过程中的铰接角稳定域与可行域，并以混合 A＊算法设计了泊车路径，但 A＊算法所需存储的数据较多（通常是指数级的），对内存依赖性较高。文［12］以 Theta＊算法设计泊车路径，利用模型预测控制（MPC）在考虑避障、时间消耗及舒适性的前提下进行泊车轨迹规划。文［13］证明了半挂汽车列车为微分平坦系统，设计了包含挂车航向角与质心位姿角反馈的微分平坦控制器，基于多项式法设计倒车轨迹，但并未针对泊车这一典型工况进行验证分析。文［14］将防折叠与防碰撞的约束条件添加到 MPC 的求解最优控制量的问题中，选择典型的 $1 / 4$ 圆与直线结合的方法构成倒车路径，

但所需车位尺寸相比于车辆本身大出许多。针对上述情况，本文以牵引挂车为研究对象，综合考虑其在泊车过程中的碰撞约束，推导出泊车起始点可行域 ${ }^{[15]}$ ，结合 B 样条曲线得到泊车路径。提出一种牵引挂车在路径跟踪过程中参考信息的选取原则，并利用 MPC 实现了车辆单步式无碰撞泊人狭窄垂直车位。

\section*{1 垂直泊车路径规划}

\section*{1.1 车辆与车位参数}

本文以 TruckSim 软件中 Cab－Over＋European Trailer 型牵引挂车为研究对象，其车轴布置形式为 S＿SS＋SSS 。为便于分析，做出以下假设：将牵引车的第2、3车轴中心取中点作为其虚拟后轴中心 $N$ ；将挂车的中间车轴中心作为其虚拟后轴中心 $M$ ，得图 1 所示车辆简化模型。铰接点 $O$ 与牵引车后轴中心 $N$ 有 3 种位置关系，分别是较接点位于牵引车后轴中心前方、后方及正上方 ${ }^{[8]}$ 。本文以铰接点位于牵引车后轴中心前方型的牵引挂车为研究对象，相关车辆参数如表 1 所示。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-02.jpg?height=479&width=757&top_left_y=1347&top_left_x=1094}
\captionsetup{labelformat=empty}
\caption{图1 车辆简化模型
Fig． 1 Simplified vehicle model}
\end{figure}

\begin{table}
\captionsetup{labelformat=empty}
\caption{表1 车辆参数
Tab． 1 Parameters of the vehicle}
\begin{tabular}{|l|l|}
\hline 参数 & 数值／m \\
\hline 牵引车轴距 $L_{0}$ & 4.135 \\
\hline 铰接点到牵引车前轴中心距离 $L_{\mathrm{a}}$ & 3.8 \\
\hline 较接点到牵引车后轴中心距离 $L_{\mathrm{b}}$ & 0.335 \\
\hline 铰接点到牵引车前端距离 $L_{0 f}$ & 5.3 \\
\hline 铰接点到挂车后轴中心距离 $L_{1}$ & 7.9 \\
\hline 挂车后悬 $L_{1 \mathrm{r}}$ & 4.2 \\
\hline 车宽 $W$ & 2.438 \\
\hline
\end{tabular}
\end{table}

根据《汽车库建筑设计规范 JGJ100－98》 ${ }^{[16]}$ 相关规定以及对比实验研究结果，选定垂直车位参数，如表 2 所示。

\section*{1.2 垂直泊车路径规划策略}

目前，在进行车辆垂直泊车路径规划时常采用

圆弧直线法，因为汽车作为一个刚体，其转弯半径很容易确定。但牵引挂车由于结构复杂，倒车时挂车的转弯半径难以确定，故以常规的圆弧直线法设计泊车路径是不理想的。考虑到牵引挂车运动时，牵引车相当于挂车的虚拟转向轮，此时的铰接角就约为挂车的虚拟转向角，本文从这一角度出发进行泊车路径规划。

\begin{table}
\captionsetup{labelformat=empty}
\caption{表2 车位参数
Tab． 2 Parameters of the parking space}
\begin{tabular}{cc}
\hline 参数 & 数值 $/ \mathrm{m}$ \\
\hline 车位长度 $P_{\mathrm{L}}$ & 19 \\
车位宽度 $P_{\mathrm{W}}$ & 4.5 \\
通车道宽度 $R_{\mathrm{W}}$ & 16 \\
\hline
\end{tabular}
\end{table}

建立以车位顶点 $P_{4}$ 为原点的车位坐标系，如图 2 所示，其中 $E 、 F 、 G 、 H$ 是以挂车最大长、宽组成的矩形框 ${ }^{[17]}$ 的顶点，$v_{1}$ 为挂车后轴中心速度，$I$ 、 $J 、 K$ 分别为挂车虚拟后轴延长线与挂车右侧边界线、左侧边界线以及通车道下边缘的交点，$O$ 为圆弧路段的转向中心。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-03.jpg?height=769&width=748&top_left_y=1420&top_left_x=223}
\captionsetup{labelformat=empty}
\caption{图2 车位坐标系
Fig． 2 Parking space coordinate system}
\end{figure}

在设计圆弧路径时考虑因素：1）当较接角大于 $90^{\circ}$ 时，车辆会发生折叠失稳；2）若以较大的较接角设计圆弧路径，车辆在跟踪时也应呈现出较大的较接角，会增加挂车后轮原地转向的概率，降低轮胎使用寿命；3）较大的较接角会使挂车泊入车位后，因牵引车难以及时摆正而造成挂车位姿再次发生变化，从而增加刮碰车位的风险。综合考虑上

述情况后，本文以 $45^{\circ}$ 铰接角作为挂车最大虚拟转向角设计圆弧路径，得挂车最小转弯半径：
$$
\begin{equation*}
R_{\min }=\frac{L_{1}}{\tan \alpha} \tag{1}
\end{equation*}
$$

式中，$\alpha$ 为挂车最大虚拟转向角；$R_{\text {min }}$ 为挂车最小转弯半径，代入参数得 $R_{\text {min }}=7.9 \mathrm{~m}$ ，此处取 8 m 。

\section*{1．2．1 车位左侧碰撞分析}

泊车时挂车最先进人车位，故对挂车的碰撞分析是至关重要的。在运动的后期（如图3所示），挂车车身与车位基本平行，为使泊车顺利进行，需保持挂车航向角为 $90^{\circ}$ ，此时牵引车的航向角也只能在 $90^{\circ}$ 附近变动。另外由于本文选取的车位比车身宽出 2 m ，所以基本排除牵引车与车位碰撞的情况。当挂车全部进人车位后，只需小幅度改变牵引车前轮转角，使其航向角迅速达到 $90^{\circ}$ ，泊车就基本完成了。故在做左侧碰撞分析时，将牵引车看作挂车的＂转向轮＂，忽略其尺寸，整个牵引挂车假定为一个乘用车，车辆避障分析如图 4 所示。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-03.jpg?height=879&width=659&top_left_y=1308&top_left_x=1139}
\captionsetup{labelformat=empty}
\caption{图3 泊车运动后期
Fig． 3 Late stage of the parking movement}
\end{figure}

此时 $H$ 点存在与车位左侧碰撞的风险，其运动轨迹近似为一个圆弧，当 $H^{\prime}$ 与转向中心 $O$ 位于同一水平位置时距车位左侧最近，有：
$$
\begin{align*}
& L_{O H}=\sqrt{\left(R_{\min }+\frac{W}{2}\right)^{2}+\left(L_{1 \mathrm{r}}\right)^{2}}  \tag{2}\\
& L_{O H^{\prime}}=P_{w}-\Delta+x_{M}  \tag{3}\\
& L_{O H}=L_{O H^{\prime}} \tag{4}
\end{align*}
$$

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-04.jpg?height=954&width=805&top_left_y=319&top_left_x=194}
\captionsetup{labelformat=empty}
\caption{图4 车位左侧临界碰撞图
Fig． 4 Critical collision diagram on the left side of parking space}
\end{figure}

式中，$L_{O H} 、 L_{O H^{\prime}}$ 分别为直线 $O H 、 O H^{\prime}$ 的长度，$x_{M}$ 为挂车后轴中心在圆弧路径起点处的横坐标，$\Delta$ 为预留安全距离，取 0.5 m 。故 $H$ 点不与车位左侧发生碰撞的临界条件为
$$
\begin{equation*}
x_{M} \geqslant \sqrt{\left(R_{\min }+\frac{W}{2}\right)^{2}+\left(L_{1 \mathrm{r}}\right)^{2}}-P_{w}+\Delta \tag{5}
\end{equation*}
$$

\section*{1．2．2 车位右侧碰撞分析}

在进行右侧碰撞分析时，依旧将牵引挂车假定为乘用车，碰撞分析如图5所示。此时有：
$$
\begin{align*}
& L_{O K}^{2}+L_{K P_{4}}^{2}=L_{O P_{4}}^{2}  \tag{6}\\
& L_{O K}=R_{\min }-y_{M}  \tag{7}\\
& L_{K P_{4}}=x_{M}  \tag{8}\\
& L_{O P_{4}}=R_{\min }-\frac{W}{2}-\Delta \tag{9}
\end{align*}
$$

式中，$L_{O K} 、 L_{K P_{4}} 、 L_{O P_{4}}$ 分别为直线 $O K 、 K P_{4} 、 O P_{4}$ 的长度，$y_{M}$ 为挂车后轴中心在圆弧路径起点处的纵坐标。

故挂车不与车位右侧发生碰撞的临界条件是：
$$
\begin{equation*}
x_{M}^{2}+\left(R_{\min }-y_{M}\right)^{2} \leqslant\left(R_{\min }-\frac{W}{2}-\Delta\right)^{2} \tag{10}
\end{equation*}
$$

\section*{1．2．3 通车道上边缘碰撞分析}

泊车过程中，最易与通车道上边缘发生碰撞的是牵引车 $A$ 点。当较接角为 $0^{\circ}$ 时，$A$ 点距通车道上

边缘最近，此时的碰撞分析如图 6 所示，其中 $A$ 、 $B 、 C 、 D$ 是以牵引车最大长、宽组成的矩形框的 4个顶点。此时有：
$$
\begin{align*}
& L_{O A}=\sqrt{\left(R_{\min }+\frac{W}{2}\right)^{2}+\left(L_{1}+L_{0 f}\right)^{2}}  \tag{11}\\
& L_{O A^{\prime}}=R_{w}-\Delta+\left(R_{\min }-y_{M}\right)  \tag{12}\\
& L_{O A}=L_{O A^{\prime}} \tag{13}
\end{align*}
$$

式中，$L_{O A} 、 L_{O A^{\prime}}$ 为直线 $O A 、 O A^{\prime}$ 的长度。故牵引挂车不与通车道上边缘碰撞的临界条件为
$$
\begin{equation*}
y_{M} \leqslant R_{w}-\Delta+R_{\min }-\sqrt{\left(R_{\min }+\frac{W}{2}\right)^{2}+\left(L_{1}+L_{0 \mathrm{f}}\right)^{2}} \tag{14}
\end{equation*}
$$

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-04.jpg?height=928&width=810&top_left_y=941&top_left_x=1064}
\captionsetup{labelformat=empty}
\caption{图5 车位右侧临界碰撞图
Fig． 5 Critical collision diagram on the right side of parking space}
\end{figure}

将相关车辆参数代入式（5）、式（10）、式（14）中，即可得圆弧路径起始点的可行区域，如图7所示。

经上述分析可知，当圆弧路径起始点位于该区域内时，均可使牵引挂车无碰撞泊人车位。但若以圆弧路径起始点作为泊车起始点，一开始就需要大幅调整牵引车航向角来实现挂车对圆弧路径的跟踪，因此可在圆弧路径起始点前加一段直线路径用于过渡。为使直线路径在具有一定过渡作用的前提下，能使整个泊车路径不过多的占用通车道，此处取为 2 m 。

\section*{1．2．4 基于 B 样条曲线的泊车路径生成}

B样条曲线具有在精确描述复杂路径形状的同

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-05.jpg?height=873&width=793&top_left_y=322&top_left_x=201}
\captionsetup{labelformat=empty}
\caption{图6 通车道上边缘临界碰撞图
Fig． 6 Upper edge of traffic lane critical collision diagram}
\end{figure}

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-05.jpg?height=662&width=768&top_left_y=1340&top_left_x=212}
\captionsetup{labelformat=empty}
\caption{图7 圆弧路径起始点可行域
Fig． 7 The drivable area at the starting point of the arc path}
\end{figure}

时又能根据曲率设计考虑整体效果的优点 ${ }^{[18]}$ 。在 1．2．3节确定的起始区域内选择圆弧路径起始点坐标为 （ $6.15,7.2$ ），相应的泊车起始点坐标为（ $8.15,7.2$ ），圆弧路径转向中心坐标为 $\left(x_{o s}, y_{o s}-R_{\text {min }}\right)$ ，其中 $x_{o s}$ 、 $y_{o s}$ 为圆弧路径起始点的横纵坐标，代入数值得转向中心坐标为（ $6.15,-0.8$ ）。为提高圆弧路径的平顺性，在其起始点与终止点之间每隔 $15^{\circ}$ 取一个控制点，各控制点的坐标可根据式（15）、式（16）求得：
$$
\begin{align*}
& x=x_{0}-R_{\min } \sin \theta  \tag{15}\\
& y=y_{0}+R_{\min } \cos \theta \tag{16}
\end{align*}
$$

式中，$x_{0} 、 y_{0}$ 为转向中心坐标，$\theta$ 为各控制点对应的圆心角。

圆弧路径终点坐标取为 $\left(-\frac{P_{w}}{2}, y_{0}\right)$ ，为使牵引挂车在泊车结束后保持良好位姿，选取泊车终止点坐标为
$$
\begin{equation*}
\left(-\frac{P_{w}}{2}, y_{P_{2}}+L_{1 r}+\Delta\right) \tag{17}
\end{equation*}
$$

式中，$y_{P_{2}}$ 为车位顶点 $P_{2}$ 在车位坐标系下的纵坐标。
引入 B 样条曲线的必过控制点特性后，共得 13 个控制点，各点坐标值如表3所示。最终的泊车路径如图8所示。

\begin{table}
\captionsetup{labelformat=empty}
\caption{表3 各控制点的坐标值
Tab． 3 Coordinate value of each control point}
\begin{tabular}{|l|l|l|l|}
\hline 控制点 & 坐标值 & 控制点 & 坐标值 \\
\hline $P_{1}$ & （8．25，7．2） & $P_{8}$ & （ $-0.78,3.2$ ） \\
\hline $P_{2}$ & （8．15，7．2） & $P_{9}$ & （－1．58，1．27） \\
\hline $P_{3}$ & （8．05，7．2） & $P_{10}$ & （－2．25，－0．8） \\
\hline $P_{4}$ & （6．15，7．2） & $P_{11}$ & （－2．25，－13．3） \\
\hline $P_{5}$ & （4．08，6．93） & $P_{12}$ & （－2．25，－14．3） \\
\hline $P_{6}$ & （2．15，6．13） & $P_{13}$ & （－2．25，－14．4） \\
\hline $P_{7}$ & （0．49，4．86） & & \\
\hline
\end{tabular}
\end{table}

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-05.jpg?height=581&width=796&top_left_y=1562&top_left_x=1071}
\captionsetup{labelformat=empty}
\caption{图8 垂直泊车路径规划图
Fig． 8 Perpendicular parking path planning diagram}
\end{figure}

\section*{2 基于牵引挂车运动学模型的 MPC}

泊车时车速一般低于 $5 \mathrm{~km} / \mathrm{h}$ ，此时可忽略轮胎侧偏与车身俯仰等动力学特性 ${ }^{[19]}$ 。假定质心侧偏角为 0 ，车辆横摆角与航向角相等，将牵引挂车简化为两轮自行车模型，如图9所示，其中，$A 、 B$ 分别为牵引车前轴中心与虚拟后轴中心，$O$ 为较接点，$C$ 为挂车虚拟后轴中心，$v_{0} 、 v_{1}$ 分别为牵引车、

挂车车速，$\varphi_{0} 、 \varphi_{1}$ 分别为牵引车、挂车航向角，$\delta$为牵引车前轮转角，$\lambda$ 为挂车虚拟转向角。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-06.jpg?height=739&width=762&top_left_y=438&top_left_x=214}
\captionsetup{labelformat=empty}
\caption{图9 2自由度自行车模型
Fig． 9 Two－degree－of－freedom bicycle model}
\end{figure}

考虑到泊车过程中挂车位姿难以控制，极易出现危险工况，故以挂车后轴中心 $C$ 为研究对象建立运动学模型，并设计基于该运动学模型的模型预测控制器。

\section*{2.1 运动学模型}

由于泊车时车速较低，假定牵引车与挂车的纵向车速相等，都取 $v$ 且不作为 MPC 的控制量。牵引车未拖带挂车时相当于一辆乘用车，其运动学模型为
$$
\left\{\begin{array}{l}
\dot{x}_{B}=v \cos \varphi_{0}  \tag{18}\\
\dot{y}_{B}=v \sin \varphi_{0} \\
\dot{\varphi_{0}}=\frac{v}{L_{0}} \tan \delta
\end{array}\right.
$$

拖带挂车后，对于挂车虚拟后轴中心 $C$ 点，有：
$$
\left\{\begin{array}{l}
\dot{x}_{C}=v \cos \varphi_{1}  \tag{19}\\
\dot{y}_{C}=v \sin \varphi_{1}
\end{array}\right.
$$

铰接点、牵引车后轴中心坐标可分别表示为
$$
\begin{align*}
& \left\{\begin{array}{l}
x_{O}=x_{C}+L_{1} \cos \varphi_{1} \\
y_{O}=y_{C}+L_{1} \sin \varphi_{1}
\end{array}\right.  \tag{20}\\
& \left\{\begin{array}{l}
x_{B}=x_{C}+L_{1} \cos \varphi_{1}-L_{b} \cos \varphi_{0} \\
y_{B}=y_{C}+L_{1} \sin \varphi_{1}-L_{b} \sin \varphi_{0}
\end{array}\right. \tag{21}
\end{align*}
$$

分别对式（20）、式（21）求导得：
$$
\begin{align*}
& \left\{\begin{array}{l}
\dot{x}_{O}=\dot{x}_{C}-\dot{\varphi}_{1} L_{1} \sin \varphi_{1} \\
\dot{y}_{O}=\dot{y}_{C}+\dot{\varphi}_{1} L_{1} \cos \varphi_{1}
\end{array}\right.  \tag{22}\\
& \left\{\begin{array}{l}
\dot{x}_{B}=\dot{x}_{C}-\dot{\varphi}_{1} L_{1} \sin \varphi_{1}+\dot{\varphi}_{0} L_{b} \sin \varphi_{0} \\
\dot{y}_{B}=\dot{y}_{C}+\dot{\varphi}_{1} L_{1} \cos \varphi_{1}-\dot{\varphi}_{0} L_{b} \cos \varphi_{0}
\end{array}\right. \tag{23}
\end{align*}
$$

因为牵引车的后轮不是转向轮，故有：
$$
\begin{equation*}
\dot{x}_{B} \sin \varphi_{0}=\dot{y}_{B} \cos \varphi_{0} \tag{24}
\end{equation*}
$$

联立式（24）、式（23）、式（19）可得：
$$
\begin{equation*}
\dot{\varphi}_{0}=\frac{\dot{\varphi}_{1} L_{1} \cos \left(\varphi_{0}-\varphi_{1}\right)-v \sin \left(\varphi_{0}-\varphi_{1}\right)}{L_{b}} \tag{25}
\end{equation*}
$$

由于挂车在铰接点处存在虚拟转向角，故有：
$$
\begin{equation*}
\dot{x}_{o} \sin \left(\varphi_{1}+\lambda\right)=\dot{y}_{o} \cos \left(\varphi_{1}+\lambda\right) \tag{26}
\end{equation*}
$$

联立式（22）、式（26）得：
$$
\begin{equation*}
\dot{\varphi}_{1}=\frac{v}{L_{1}} \tan \lambda \tag{27}
\end{equation*}
$$

由式（18）可知：
$$
\begin{equation*}
\delta=\arctan \left(\dot{\varphi}_{0} \frac{v}{L_{0}}\right) \tag{28}
\end{equation*}
$$

联立式（25）、式（27）、式（28）得：
$$
\begin{equation*}
\delta=\arctan \left(\frac{L_{0}}{L_{b}}\left(\tan \lambda \cos \left(\varphi_{0}-\varphi_{1}\right)-\sin \left(\varphi_{0}-\varphi_{1}\right)\right)\right) \tag{29}
\end{equation*}
$$

也就有：
$$
\begin{equation*}
\tan \lambda=\frac{\tan \delta L_{b}}{L_{0} \cos \left(\varphi_{0}-\varphi_{1}\right)}+\tan \left(\varphi_{0}-\varphi_{1}\right) \tag{30}
\end{equation*}
$$

将式（30）代人式（27）得：
$$
\begin{equation*}
\dot{\varphi}_{1}=\frac{v}{L_{1}}\left(\frac{L_{b} \tan \delta}{L_{0} \cos \left(\varphi_{0}-\varphi_{1}\right)}+\tan \left(\varphi_{0}-\varphi_{1}\right)\right) \tag{31}
\end{equation*}
$$

式（19）、式（25）、式（31）即为所需车辆运动学模型，系统的状态量为 $\boldsymbol{X}(t)=\left[x_{C}, y_{C}, \varphi_{0}, \varphi_{1}\right]^{\mathrm{T}}$ ，控制量为 $u(t)=\delta$ 。

\section*{2.2 预测模型的推导}
2.1 节所得车辆运动学模型的状态空间表达式为
$$
\begin{equation*}
\dot{\boldsymbol{X}}(t)=\boldsymbol{f}(\boldsymbol{X}(t), u(t)) \tag{32}
\end{equation*}
$$

式中， $\boldsymbol{f}$ 为系统的状态转移函数。该模型为非线性时变的连续模型，为提高控制器的实时性，利用泰勒级数与一阶差商法对式（32）进行线性化与离散化处理 ${ }^{[20]}$ ，得线性时变离散方程：
$$
\begin{equation*}
\tilde{\boldsymbol{X}}(k+1 \mid t)=\boldsymbol{a}(k \mid t) \tilde{\boldsymbol{X}}(k \mid t)+\boldsymbol{b}(k \mid t) \tilde{u}(k \mid t) \tag{33}
\end{equation*}
$$

式中，
$$
\begin{gather*}
\boldsymbol{a}(k \mid t)=\left[\begin{array}{cccc}
1 & 0 & 0 & -v \sin \varphi_{1 \_ \text {ref }} T \\
0 & 1 & 0 & v \cos \varphi_{1 \_ \text {ref }} T \\
0 & 0 & 1 & 0 \\
0 & 0 & P Q T & 1-P Q T
\end{array}\right]  \tag{34}\\
\boldsymbol{b}(k \mid t)=\left[\begin{array}{c}
0 \\
0 \\
\frac{v}{L_{0} \cos ^{2} \delta_{\_ \text {ref }}} T \\
\frac{v}{L_{1}}\left(\frac{L_{b}}{L_{0} \cos \left(\varphi_{0 \_ \text {ref }}-\varphi_{1 \_ \text {ref }}\right) \cos ^{2} \delta_{\_ \text {ref }}}\right) T
\end{array}\right] \tag{35}
\end{gather*}
$$

其中，$P=\frac{v}{L_{1} \cos ^{2}\left(\varphi_{0 \text {＿ref }}-\varphi_{1 \text {＿ref }}\right)}, T$ 为采样时间，$Q= \frac{\tan \delta_{\_ \text {ref }} L_{b} \sin \left(\varphi_{0 \_ \text {ref }}-\varphi_{1 \_ \text {ref }}\right)}{L_{0}}+1, \widetilde{u}(k \mid t)=\delta-\delta_{\text {ref }}$, $\tilde{\boldsymbol{X}}(k \mid t)=\left[\begin{array}{c}x_{C}-x_{C \_\mathrm{ref}} \\ y_{C}-y_{C \_\mathrm{ref}} \\ \varphi_{0}-\varphi_{0 \_\mathrm{ref}} \\ \varphi_{1}-\varphi_{1 \_\mathrm{ref}}\end{array}\right]$ 。

为避免前轮转角剧烈突变导致车辆无法正常行驶，常对控制量 $\delta$ 的增量 $\Delta \delta$ 进行限制 ${ }^{[21]}$ ，令：
$$
\boldsymbol{\lambda}(k \mid t)=\left[\begin{array}{c}
\tilde{\boldsymbol{X}}(k \mid t)  \tag{36}\\
\tilde{u}(k-1 \mid t)
\end{array}\right]
$$

通过式（33）、式（36）推导预测模型状态空间表达式：
$$
\begin{equation*}
\boldsymbol{\lambda}(k+1 \mid t)=\overline{\boldsymbol{A}}(k) \boldsymbol{\lambda}(k \mid t)+\overline{\boldsymbol{B}}(k) \Delta \tilde{u}(k \mid t) \tag{37}
\end{equation*}
$$

式中，$\overline{\boldsymbol{A}}(k \mid t)=\left[\begin{array}{cc}\boldsymbol{a}(k \mid t) & \boldsymbol{b}(k \mid t) \\ \mathbf{0}_{m \times n} & \boldsymbol{I}_{m}\end{array}\right], \quad \overline{\boldsymbol{B}}(k \mid t)= \left[\begin{array}{c}\boldsymbol{b}(k \mid t) \\ \boldsymbol{I}_{m}\end{array}\right], n 、 m$ 分别为状态矩阵与控制矩阵的维数。

建立预测模型输出方程：
$$
\boldsymbol{y}(k \mid t)=\left[\begin{array}{ll}
\boldsymbol{I}_{n} & \mathbf{0}
\end{array}\right]\left[\begin{array}{c}
\tilde{\boldsymbol{X}}(k \mid t)  \tag{38}\\
\tilde{u}(k-1 \mid t)
\end{array}\right]
$$

令 $\boldsymbol{C}(k)=\left[\begin{array}{ll}\boldsymbol{I}_{n} & \mathbf{0}\end{array}\right]$ ，并设定预测时域为 $N_{\mathrm{p}}$ 、控制时域为 $N_{\mathrm{c}}\left(N_{\mathrm{p}}>N_{\mathrm{c}}\right)$ ，即可得未来 $N_{\mathrm{c}}$ 个时域的控制量作用于系统时，之后 $N_{\mathrm{p}}$ 个时域预测模型的输出表达式：
$$
\begin{equation*}
\boldsymbol{Y}(t)=\boldsymbol{A}_{t} \boldsymbol{\lambda}(t \mid t)+\boldsymbol{B}_{t} \Delta \boldsymbol{U}(t) \tag{39}
\end{equation*}
$$

式中，
$\boldsymbol{Y}(t)=\left[\begin{array}{c}\boldsymbol{y}(t+1 \mid t) \\ \boldsymbol{y}(t+2 \mid t) \\ \vdots \\ \boldsymbol{y}\left(t+N_{\mathrm{c}} \mid t\right) \\ \vdots \\ \boldsymbol{y}\left(t+N_{\mathrm{p}} \mid t\right)\end{array}\right], \boldsymbol{A}=\left[\begin{array}{c}\boldsymbol{C} \overline{\boldsymbol{A}}_{t} \\ \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{2} \\ \vdots \\ \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{c}}} \\ \vdots \\ \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{p}}}\end{array}\right], \Delta \boldsymbol{U}=\left[\begin{array}{c}\tilde{u}(t \mid t) \\ \tilde{u}(t+1 \mid t) \\ \tilde{u}(t+2 \mid t) \\ \vdots \\ \tilde{u}\left(t+N_{c} \mid t\right)\end{array}\right]$
$\boldsymbol{B}=\left[\begin{array}{ccccc}\boldsymbol{C} \overline{\boldsymbol{B}}_{t} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\ \boldsymbol{C} \overline{\boldsymbol{A}}_{t} \overline{\boldsymbol{B}}_{t} & \boldsymbol{C} \overline{\boldsymbol{B}}_{t} & \mathbf{0} & \cdots & \mathbf{0} \\ \vdots & \vdots & \vdots & \ddots & \vdots \\ \overline{\boldsymbol{C A}}_{t}^{N_{\mathrm{c}}-1} \overline{\boldsymbol{B}}_{t} & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{c}}-2} \overline{\boldsymbol{B}}_{t} & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{c}}-3} \overline{\boldsymbol{B}}_{t} & \cdots & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{0} \overline{\boldsymbol{B}}_{t} \\ \vdots & \vdots & \vdots & \ddots & \vdots \\ \overline{\boldsymbol{C A}}_{t}^{N_{\mathrm{p}}-1} \overline{\boldsymbol{B}}_{t} & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{p}}-2} \overline{\boldsymbol{B}}_{t} & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{p}}-3} \overline{\boldsymbol{B}}_{t} & \cdots & \boldsymbol{C} \overline{\boldsymbol{A}}_{t}^{N_{\mathrm{p}}-N_{\mathrm{c}}-1} \overline{\boldsymbol{B}}_{t}\end{array}\right]$

\section*{2.3 目标函数的设计}

泊车过程中，需要将安全性与稳定性作为优化目标。安全性是指车辆对参考路径的跟踪精度，稳定性指跟踪过程中前轮转角增量的大小 ${ }^{[22]}$ 。基于此，设计目标函数为
$$
\begin{align*}
\boldsymbol{J}(t)= & \sum_{i=1}^{N_{\mathrm{p}}}\left\|\boldsymbol{y}(k+i \mid t)-\boldsymbol{y}_{\mathrm{ref}}(k+i \mid t)\right\|_{\boldsymbol{Q}}^{2}+ \\
& \sum_{i=1}^{N_{\mathrm{c}}-1}\|\Delta \tilde{u}(k+i \mid t)\|_{\boldsymbol{R}}^{2}+\rho \varepsilon^{2} \tag{40}
\end{align*}
$$

式中， $\boldsymbol{Q} 、 \boldsymbol{R}$ 分别为安全性权重增益与稳定性权重增益； $\boldsymbol{y}_{\mathrm{ref}}(k+i \mid t)=[0,0, \cdots, 0]^{\mathrm{T}}$ 为参考输出量； $\rho$ 为权重系数；$\varepsilon$ 为松他因子。等式右侧的第1项反映了车辆对参考路径的跟踪能力；第 2 项反映了车辆跟踪过程中的稳定性；第 3 项可避免待优化问题在规定时域内无可行解的情况。

\section*{2.4 约束条件的建立}

控制过程中控制量约束与控制增量约束可表示为
$$
\left\{\begin{array}{l}
\widetilde{u}_{\min }(t+k) \leqslant \widetilde{u}(t+k) \leqslant \widetilde{u}_{\max }(t+k)  \tag{41}\\
\Delta \widetilde{u}_{\min }(t+k) \leqslant \Delta \widetilde{u}(t+k) \leqslant \Delta \widetilde{u}_{\max }(t+k)
\end{array}\right.
$$

式中，$\tilde{u}_{\text {max }}(t+k) 、 \tilde{u}_{\text {min }}(t+k)$ 分别为 $t+k$ 时刻控制量的上下限；$\Delta \widetilde{u}_{\text {max }}(t+k) 、 \Delta \widetilde{u}_{\text {min }}(t+k)$ 分别为 $t+k$时刻控制增量的上下限。二者存在关系：
$$
\begin{equation*}
\widetilde{u}(t+k)=\widetilde{u}(t+k-1)+\Delta \widetilde{u}(t+k) \tag{42}
\end{equation*}
$$

对未来 $N_{\mathrm{c}}$ 个控制时域有：
$$
\begin{equation*}
\boldsymbol{U}_{\min } \leqslant \boldsymbol{U}_{t}+\boldsymbol{I} \Delta \boldsymbol{U}_{t} \leqslant \boldsymbol{U}_{\max } \tag{43}
\end{equation*}
$$

式中，
$\boldsymbol{U}_{t}=\left[\begin{array}{c}\tilde{u}(k-1 \mid t) \\ \tilde{u}(k-1 \mid t) \\ \vdots \\ \tilde{u}(k-1 \mid t)\end{array}\right], \boldsymbol{I}=\left[\begin{array}{cccc}\boldsymbol{I}_{m} & \mathbf{0} & \cdots & \mathbf{0} \\ \boldsymbol{I}_{m} & \boldsymbol{I}_{m} & \cdots & \mathbf{0} \\ \vdots & \vdots & \ddots & \vdots \\ \boldsymbol{I}_{m} & \boldsymbol{I}_{m} & \cdots & \boldsymbol{I}_{m}\end{array}\right], \Delta \boldsymbol{U}_{t}= \left[\begin{array}{c}\Delta \tilde{u}(k \mid t) \\ \Delta \tilde{u}(k+1 \mid t) \\ \vdots \\ \Delta \tilde{u}\left(k+N_{\mathrm{c}}-1 \mid t\right)\end{array}\right]$

最终得约束条件：
$$
\left\{\begin{array}{l}
\Delta \boldsymbol{U}_{\min } \leqslant \Delta \boldsymbol{U}_{t} \leqslant \Delta \boldsymbol{U}_{\max }  \tag{44}\\
\boldsymbol{U}_{\min } \leqslant \boldsymbol{I} \Delta \boldsymbol{U}_{t}+\boldsymbol{U}_{t} \leqslant \boldsymbol{U}_{\max }
\end{array}\right.
$$

将求解出的控制序列中第 1 个元素 $\Delta \tilde{u}_{t}{ }^{*}$ 作为实际控制增量作用于系统，循环求解，滚动优化，实现对参考路径的跟踪。

\section*{2.5 参考信息选取原则}

通过前面推导可知：利用 MPC 控制车辆运动时，需要明确的参数包括实际状态量与控制量、参

考状态量与控制量。其中，实际状态量与控制量可利用车辆自身传感器获得，参考信息选取方法直接影响控制器的控制效果。

给牵引挂车一恒定前轮转角（ $5^{\circ}$ ）与恒定车速 （ $-2 \mathrm{~km} / \mathrm{h}$ ）使其倒车行驶。分别将挂车后轴中心、铰接点与牵引车后轴中心的实际运动轨迹绘制出来，如图 10 所示。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-08.jpg?height=883&width=770&top_left_y=717&top_left_x=210}
\captionsetup{labelformat=empty}
\caption{图 10 实际运动轨迹
Fig． 10 Actual motion trajectory}
\end{figure}

由于本节的重点是实现对挂车的准确控制，故将图中挂车后轴中心实际运动轨迹假定为挂车的参考路径进行分析。路径跟踪的最理想情况是：在 $t$时刻，挂车后轴中心位于参考路径上某一点（图10中蓝点）且其航向与该点相切；牵引车航向应与 $t$ 时刻对应的 $\left(x_{B}, y_{B}\right)$（图10中红点）相切，由式（21）、式（25）可看出 $\varphi_{0}$ 与 $\varphi_{1} 、\left(x_{B}, y_{B}\right)$ 间存在耦合关系，利用与 $\left(x_{B}, y_{B}\right)$ 点相切或通过式（25）利用 $\varphi_{1}$ 推导 $\varphi_{0}$均难以得到 $\varphi_{0}$ 。但此时较接点的坐标可根据式（20）求得，考虑到铰接点既位于挂车上，又位于牵引车上，从图 10 可以看到较接点与牵引车后轴中心的实际运动轨迹基本重合，故近似认为此时牵引车的航向与 $\left(x_{O}, y_{O}\right)$ 。 图10中黑点）相切。

所以提出参考信息选取原则为：将第1节所得泊车路径作为挂车的参考路径，将其离散成 $N$ 个参考点，寻找当前时刻 $(t)$ 距离挂车后轴中心最近的参考点坐标 $x_{\text {r＿C }}(i) 、 y_{\text {r＿C }}(i)(i \in\{1,2, \cdots, N\})$ 作为参考横纵坐标；挂车参考航向应与该点相切，鉴

于参考路径为不规则曲线，选择 $\left(x_{\text {r＿C }}(i), y_{\text {r＿C }}(i)\right)$ 、 （ $\left.x_{\text {r＿C }}(i-1), ~ y_{\text {r＿C }}(i-1)\right)$ 两点的斜率方向近似为挂车参考航向，其航向角计算公式为
$$
\begin{equation*}
\varphi(i)_{1 \_ \text {ref }}=\arctan \frac{y_{\text {r_C }}(i)-y_{\text {r_C }}(i-1)}{x_{\text {r_C }}(i)-x_{\text {r_C }}(i-1)} \tag{45}
\end{equation*}
$$

当挂车按泊车路径行驶时，铰接点的理想运动轨迹点（ $x_{r_{-} O}(i) 、 y_{r_{-} O}(i)$ ）可通过式（20）求得，故牵引车参考航向角可近似计算为
$$
\begin{equation*}
\varphi(i)_{0 \_ \text {ref }}=\arctan \frac{y_{\mathrm{r}_{-} O}(i)-y_{\mathrm{r}_{-} O}(i-1)}{x_{\mathrm{r}_{-} O}(i)-x_{\mathrm{r}_{-} O}(i-1)} \tag{46}
\end{equation*}
$$

各个参考点对应的参考前轮转角可根据式（28）求得；或利用式（21）求得各个（ $x_{\text {r＿C }}(i), y_{\text {r＿C }}(i)$ ）对应的（ $x_{\text {r＿B }}(i), y_{\text {r＿B }}(i)$ ）后，再由阿克曼转向原理求得。这样，每个参考点携带 5 个参考信息（ $x_{\mathrm{r}_{-} \mathrm{C}}$ ， $y_{\text {r＿C }}, \varphi_{0_{\text {＿ref }}}, \varphi_{1_{\text {＿ref }}}, \delta_{\text {ref }}$ ）输入给 MPC。

\section*{3 仿真分析}

本节主要利用 Matlab 与 TruckSim 进行联合仿真验证，通过多次仿真实验，选取了 4 种具有代表性的工况进行分析。由于选择的工况是车辆从车位右侧开始泊车，所以仿真图中的最右侧位置为开始位置。

\section*{3．1 Matlab 仿真分析}

首先，利用 MPC 控制车辆跟踪第1节规划出的泊车路径，得挂车后轴中心实际运动轨迹曲线如图11（a）所示。为验证提出的路径规划方法是否扩大了泊车起始点的可选范围，采用相同宽度的车位，在可行域内通过改变泊车起始点的纵向位置再找一点（ $8.15,6.8$ ）作为挂车后轴中心的泊车起始点进行仿真，结果如图11（b）所示。

前面提到的有关泊车路径规划与运动控制的文献中，采用的车位最小宽度为 6 m ，为验证提出的路径规划方法对于不同尺寸车位是否适用，选用新的车位尺寸，如表 4 所示。

根据第1节提出的路径规划方法，得圆弧路径起始点可行域，如图 12 所示。

为便于对比分析，从该区域内选择与图11（a）相同的泊车起始点（8．15，7．2）以及改变其横向位置的起始点（7．9，7．2），进行仿真验证，结果如图13所示。

在图11、图13中，红色曲线为参考路径，蓝色曲线为挂车后轴中心实际运动轨迹曲线。从图中可以看出，由于在圆弧路径前添加了一段用于过渡的直线路径，挂车从起泊点运动到圆弧路径起始点的过程中，通过调整前轮转角，牵引车与挂车之间的

铰接角已达到一定数值，即挂车的虚拟转向角已足以支撑其实现圆弧路径的跟踪。但在泊人车位时，由于圆弧路径结束点与一段直线路径连接，二者间存在曲率阶跃，这就需要较接角从一定数值快速调整为 0 ，但较接角的调整依赖于牵引车的前轮转角，且前轮转角及其增量存在极限约束，使得较接角的变化存在一定的迟滞性，导致挂车在泊人车位时存在跟踪误差。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-09.jpg?height=1274&width=793&top_left_y=769&top_left_x=201}
\captionsetup{labelformat=empty}
\caption{图11 车宽为 4.5 m 时的轨迹跟踪图
Fig． 11 Trajectory tracking diagram with a vehicle width of 4.5 m}
\end{figure}

\begin{table}
\captionsetup{labelformat=empty}
\caption{表4 对比车位的车位参数
Tab． 4 The parameters of the constrasting parking space}
\begin{tabular}{cc}
\hline 参数 & 数值 $/ \mathrm{m}$ \\
\hline 车位长度 $P_{\mathrm{L}}$ & 19 \\
车位宽度 $P_{\mathrm{W}}$ & 5 \\
通车道宽度 $R_{\mathrm{W}}$ & 16 \\
\hline
\end{tabular}
\end{table}

对比图11（a）和图11（b）可以看出，在相同车位尺寸对应的泊车起始点可行域中选择不同纵坐标

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-09.jpg?height=671&width=775&top_left_y=319&top_left_x=1080}
\captionsetup{labelformat=empty}
\caption{图12 圆弧路径起始点可行域
Fig． 12 The drivable area at the starting point of the arc path}
\end{figure}

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-09.jpg?height=1278&width=791&top_left_y=1142&top_left_x=1071}
\captionsetup{labelformat=empty}
\caption{图13 车宽为 5 m 时的轨迹跟踪图
Fig． 13 Trajectory tracking diagram with a vehicle width of 5 m}
\end{figure}

的起泊点，挂车后轴中心的跟踪效果相差不大。对比图11（a）与图13（a）可看出，对于不同车位尺寸选择相同的起泊点，跟踪效果有差异，前者的泊车

效果更好。可以观察到，在 4.5 m 宽的车位中，起泊点位于可行域横向中间范围，而在 5 m 宽的车位中，起泊点临近可行域横向右侧边界位置。图13（b）中选用了与图13（a）纵坐标相同，但其横坐标属于该可行域横向中间范围，跟踪效果优于图13（a）。即在可行域内对于起泊点横坐标的选取影响着跟踪效果的好坏，泊车时应尽可能地选取可行域横向中间范围内的点作为泊车起始点。

\section*{3．2 Simulink 与 TruckSim 联合仿真}

TruckSim 软件中提供的车辆模型更接近实车运动状况，为观察泊车过程中车辆的位姿变化、车辆与车位间是否发生碰撞以及验证本文方法的可行性，分别针对以上 4 种工况进行 TruckSim 与 Simulink 联合仿真。

以第 1 节生成的泊车路径作为参考路径，其仿真动画如图 14 所示。设定牵引挂车的初始航向角在泊车开始时为 0 ，如图14（a）；由图14（b）、图14（c）可以看出车辆在泊车过程中并未擦碰车道线；图 14（d）、图14（e）反映出车辆在进人车位时与其两侧保持了一定的安全距离；从图14（f）可以看到，车辆在泊车结束后可以保持良好的车姿。

图15反映的是通过改变泊车起始点位置与改变车位尺寸后的联合仿真动画。从图中可以看到，在更换起泊点坐标位置与改变车位尺寸后，牵引挂车仍然可以实现安全无碰撞的单步式泊人车位。观察泊车过程图可以看到，泊车开始后，牵引车通过改变前轮转角使整车较接角增大，以满足挂车跟踪圆弧路径的需求，进一步验证了上文的分析。证明了本文提出的基于牵引挂车泊车碰撞约束的路径规

划方法可以使车辆达到安全无碰撞的泊车效果；提出的以 $45^{\circ}$ 铰接角作为挂车虚拟转向角设计的圆弧路径可以与牵引挂车的跟踪能力相匹配。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-10.jpg?height=1317&width=823&top_left_y=495&top_left_x=1064}
\captionsetup{labelformat=empty}
\caption{图 14 联合仿真动画
Fig． 14 Co－simulation animation}
\end{figure}

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-10.jpg?height=681&width=1693&top_left_y=1928&top_left_x=185}
\captionsetup{labelformat=empty}
\caption{图 15 更改泊车起始点与车位尺寸的泊车动画
Fig． 15 Parking animation for changing parking start point and parking space size}
\end{figure}

图16反映的是在以上4种工况的联合仿真实验中，挂车后轴中心对于参考路径的跟踪误差。从图中可以看到，泊人车位前由于直线路径的过渡作用，使得牵引挂车对于圆弧路径的跟踪效果良好。但进入车位时，由于圆弧路径终点与直线路径间的曲率阶跃，使得此处的跟踪误差较大。观察其峰值可以看出，同一车位中，改变起泊点的纵坐标，其误差峰值无论是出现的位置或峰值大小都相差甚微；针对不同车位尺寸，当起泊点均位于其可行域横向中间范围时，峰值大小相差不大，只是出现的位置有所偏差。但当起泊点不在中间范围时，误差峰值相对较大。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-11.jpg?height=593&width=791&top_left_y=991&top_left_x=203}
\captionsetup{labelformat=empty}
\caption{图16 跟踪误差图
Fig． 16 Tracking error diagram}
\end{figure}

以上 4 种工况中，跟踪误差的最大值约为 0.45 m ，但其选用的车位比车辆本身宽出 2.5 m ，并且从仿真效果图来看，峰值误差的出现并未使车辆出现不可控现象，而且跟踪误差很快收敛于 0 ，证明所提出的 MPC 参考信息选取原则使其具有良好的控制效果。

图 17 中反映的是牵引挂车在针对以上 4 种工况的联合仿真过程中，铰接角的变化情况。从图中可以看出，在车辆到达圆弧路径起始点时，铰接角已达到 $-30^{\circ}$ 左右，所以在圆弧路径前的直线过渡路径是非常有必要的。整个过程中的铰接角最大值约为 $-40^{\circ}$ ，即车辆在泊车过程中并未出现折叠失

稳的情况，证明了本文提出的牵引挂车单步式泊车规划控制方案的可行性。

\begin{figure}
\includegraphics[alt={},max width=\textwidth]{https://cdn.mathpix.com/cropped/02cc437c-f5af-4d7f-9aa3-8c53bfdbed9e-11.jpg?height=771&width=802&top_left_y=447&top_left_x=1069}
\captionsetup{labelformat=empty}
\caption{图17 泊车过程中铰接角的变化情况
Fig． 17 Variation of the hinge angle during parking}
\end{figure}

\section*{4 结论}

针对牵引挂车在泊车过程中挂车位姿难以控制的问题，提出以铰接角作为挂车虚拟转向角，分析其在泊车过程中的碰撞情况，求解圆弧路径起始点的理论可行域，利用 B 样条曲线与圆弧直线法设计泊车路径；以挂车后轴中心为研究对象推导运动学模型，基于此设计模型预测控制器，针对控制器的输人参考信息提出了一种选取原则。通过 TruckSim与 Simulink 联合仿真，验证了本文提出的泊车路径规划方法可用于牵引挂车单步式垂直泊车，并能降低对泊车起始点的精度要求，提出的模型预测控制器参考信息选取原则使其具有良好的控制效果。

未来的改进方向包括：1）选用的车位尺寸仍未达到《汽车库建筑设计规范》要求，后续可考虑利用多步式泊人的方法进一步降低对于车位尺寸的要求。2）将速度控制加人到 MPC 中。3）从图 17 可以看到，泊车过程中存在几处较接角突变，后续可以考虑对泊车路径进一步进行平滑处理。

\section*{参考文献}
［1］邵俊恺，赵咧，杨珏，等．无人驾驶铰接式车辆强化学习路径跟踪控制算法［J］．农业机械学报，2017，48（3）：376－ 382.

SHAO J K，ZHAO X，YANG Y，at el．Reinforcement learning algorithm for path following control of articulated vehicle［J］．
Transactions of the Chinese Society for Agricultural Machinery，2017，48（3）：376－382．
［2］丁庆国．牵引－半挂车右转弯后视盲区研究［J］．林业机械与木工设备，2021，49（2）：58－61．