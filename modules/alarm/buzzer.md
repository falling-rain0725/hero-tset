# 蜂鸣器音乐报警模块

本模块使用无源蜂鸣器播放音乐，用不同音乐表示不同故障。播放器采用非阻塞状态机，不使用 `HAL_Delay()`，因此不会长时间占用 FreeRTOS 任务。

当前内置《春日影》、《Bad Apple!!》、《帝国进行曲》、《See You Again》短版和三声启动音，并通过优先级决定多个报警同时出现时播放哪一首。

## 文件组成

```text
modules/alarm/buzzer.c        播放状态机、续期、优先级选择和PWM输出
modules/alarm/buzzer.h        对外接口与音乐编号
modules/alarm/buzzer_music.c  音符频率表与乐谱数据
modules/alarm/buzzer_music.h  音符定义与乐谱声明
bsp/pwm/bsp_pwm.c             通用PWM底层驱动
```

## 工作原理

完整调用链如下：

```text
故障检测
  → BuzzerPlay()
  → 激活音乐或刷新有效期
  → BuzzerTask()
  → 读取当前音符和持续时间
  → PWMSetPeriod() / PWMSetDutyRatio()
  → 蜂鸣器发声
```

蜂鸣器使用 TIM4 CH3 输出 PWM：

- PWM 频率决定音高；
- PWM 占空比影响音量；
- 每首音乐拥有独立的基础时间单位；
- `BuzzerTask()` 当前每 10 ms 调用一次；
- DWT 浮点时间用于计算音乐有效期和音符结束时间；
- 播放过程不阻塞其他任务。

## 接口使用

### 初始化

```c
BuzzerInit();
```

初始化 PWM 和播放器状态。当前实现带有重复初始化保护，多次调用不会重复注册同一个 PWM 通道。

### 播放或续期

```c
BuzzerPlay(BUZZER_MUSIC_HARUHIKAGE, 1.0f);
```

含义是播放《春日影》，并保证它在最后一次调用后的 1 秒内保持有效。

同一首音乐已经播放时，再次调用只刷新截止时间，不会：

- 重新播放第一个音符；
- 累加剩余播放时长；
- 打断当前音乐节奏。

### 无限播放

```c
BuzzerPlay(
    BUZZER_MUSIC_HARUHIKAGE,
    BUZZER_PLAY_FOREVER
);
```

`BUZZER_PLAY_FOREVER` 的值为 `-1.0f`。进入无限播放后，必须显式停止。

### 完整播放一次

```c
BuzzerPlay(
    BUZZER_MUSIC_SEE_YOU_AGAIN,
    BUZZER_PLAY_ONCE
);
```

`BUZZER_PLAY_ONCE` 的值为 `-2.0f`。播放器读到乐谱末尾的 `0xFF` 后自动停止，不需要计算歌曲总时长。同一首音乐尚未播放结束时重复调用，不会从头播放。

### 停止指定音乐

```c
BuzzerStop(BUZZER_MUSIC_HARUHIKAGE);
```

也可以将播放时间传入 `0.0f`：

```c
BuzzerPlay(BUZZER_MUSIC_HARUHIKAGE, 0.0f);
```

### 停止全部音乐

```c
BuzzerStopAll();
```

### 查询播放状态

```c
if (BuzzerIsPlaying(BUZZER_MUSIC_HARUHIKAGE))
{
    // 当前音乐仍处于活动状态
}
```

### 周期任务

```c
BuzzerTask();
```

必须周期调用该函数才能推进乐谱。当前工程在 Daemon 任务中每 10 ms 调用一次。

## 续期机制

有限播放采用“最后一次调用后继续播放指定时长”的机制。

假设上层每次请求播放 1 秒：

```text
0.0 s 调用 → 截止时间为 1.0 s
0.5 s 调用 → 截止时间刷新为 1.5 s
1.0 s 调用 → 截止时间刷新为 2.0 s
停止调用   → 最后一次调用的 1 秒后停止
```

它不是在剩余时间上继续累加。否则上层调用频率越高，未播放的时间就会积压，故障恢复后仍可能长时间鸣叫。

时间参数定义如下：

| 参数 | 行为 |
|---|---|
| `duration_s > 0.0f` | 最后一次调用后继续播放指定秒数 |
| `duration_s == 0.0f` | 停止指定音乐 |
| `duration_s == BUZZER_PLAY_FOREVER` | 无限播放，直到显式停止 |
| 其他负数 | 非法参数，忽略 |

## 当前 CAN 掉线报警

CANComm 的离线回调当前使用《帝国进行曲》短版：

```c
BuzzerPlay(BUZZER_MUSIC_IMPERIAL_MARCH, 1.0f);
```

Daemon 检测到通信离线后，会周期执行该回调：

- 离线期间不断刷新 1 秒有效期，音乐连续播放且不会重新开始；
- 收到合法 CAN 数据后，Daemon 被重新加载，离线回调停止；
- 最后一次离线上报的 1 秒后，音乐自动停止。

当前恢复回调不需要显式调用 `BuzzerStop()`。

## 乐谱数据格式

乐谱使用压缩的“音符、持续时间”格式：

```c
const uint8_t buzzer_music_haruhikage[] = {
    NOTE_E6, 8,
    NOTE_D6, 4,
    NOTE_C6, 8,
    NOTE_D6, 4,

    0xFF
};
```

其中：

- 第一个字节是音符编号；
- 第二个字节是持续时间单位；
- 一个时间单位由对应音乐配置的 `note_unit_ms` 定义；
- `NOTE_P` 表示休止；
- `0xFF` 表示乐谱结束；
- 音乐仍有效时，播放到 `0xFF` 会自动从头循环。

《春日影》当前包含 397 个音符或休止符：

- 压缩乐谱占 795 B；
- 共用音符频率表占 178 B；
- 完整播放一遍约 113 秒。

当前音乐的时间单位为：

| 音乐 | `note_unit_ms` | 说明 |
|---|---:|---|
| 《春日影》 | 62.5 ms | 保持原乐谱速度不变 |
| 《Bad Apple!!》 | 54.3478 ms | 138 BPM 下的 1/8 拍 |
| 《帝国进行曲》短版 | 100 ms | 18 个音符，完整播放约 8 秒 |
| 《See You Again》 | 96.153846 ms | 78 BPM 下每拍八等分 |
| 《See You Again》二倍速 | 48 ms | 与原版共用乐谱，仅改变基础时间单位 |
| 三声启动音 | 50 ms | C5-E5-G5，适合使用单次播放 |

## 添加新音乐

添加音乐时不要先修改上层报警调用。应先准备单声部乐谱、接入播放器并使用 `BUZZER_PLAY_ONCE` 独立试听，确认听感后再决定它对应哪一种故障。

完整修改范围如下：

```text
buzzer.h        增加音乐编号
buzzer_music.h  声明乐谱数组
buzzer_music.c  保存音符和持续时间
buzzer.c        配置速度、优先级和音量
上层模块        测试完成后按需调用，可暂时不改
```

### 1. 准备单声部旋律

无源蜂鸣器同一时刻只能播放一个音，不能直接还原钢琴和弦或多轨 MIDI。选谱时优先使用辨识度最高的主旋律，并进行以下处理：

- 删除和弦、伴奏、鼓点和重叠音符；
- 将超出蜂鸣器有效范围的音符移高或移低八度；
- 使用 `NOTE_P` 保留必要的停顿；
- 重复音之间留出短暂间隔，避免听起来像一个长音；
- 先截取有辨识度的短片段，不要一开始就放入完整歌曲。

乐谱准备实际使用过两种方法：从现成多轨 MIDI 提取旋律，以及根据字母谱手工录入旋律。转换脚本统一放在工程的 `my_log` 中，不加入 `modules/alarm`，避免临时 MIDI、CSV 和 Python 文件参与固件编译。

#### 方法一：从现成 MIDI 提取，以《Bad Apple!!》为例

工作目录：

```text
E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\bad_apple_score
```

转换脚本：

```text
E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\bad_apple_score\extract_bad_apple.py
```

相关文件：

```text
bad_apple_source.mid           原始多轨MIDI
bad_apple_melody.mid           提取后的单声部试听文件
bad_apple_melody.csv           音符、起止时间和量化时值明细
bad_apple_score_preview.c.txt  自动生成的C数组预览
```

脚本使用 Python 的 `mido` 库完成以下处理：

1. 打开原始 MIDI，检查每条轨道中的音符数量和乐器内容；
2. 选择主旋律所在轨道，《Bad Apple!!》当前选择的是脚本中的 `MELODY_TRACK = 6`；
3. 累加 MIDI delta time，得到每个 `note_on` 和 `note_off` 的绝对 tick；
4. 将同一音符的开始与结束事件配对，得到音高、开始时间和持续时间；
5. 根据相邻音符的时间空隙自动插入 `NOTE_P`；
6. 按每拍八等分进行节奏量化，即脚本中的 `SUBDIVISIONS_PER_BEAT = 8`；
7. 将 MIDI 音高转换成工程音符名，例如 MIDI 69 转成 `NOTE_A4`；
8. 将超过 `255` 个单位的长音或长休止拆成多个条目；
9. 同时输出单声部 MIDI、CSV 和 C 数组预览。

首次使用需要安装依赖：

```powershell
python -m pip install mido
```

重新生成：

```powershell
cd E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\bad_apple_score
python extract_bad_apple.py
```

先播放 `bad_apple_melody.mid`。确认选中的确实是主旋律、速度正常且没有重叠音后，再从 `bad_apple_score_preview.c.txt` 复制数组到 `buzzer_music.c`。

#### 方法二：根据乐谱手工录入，以《See You Again》为例

《See You Again》没有直接使用来源 MIDI，而是先参考 F 大调字母谱和钢琴谱，手工整理副歌人声旋律，再由脚本生成试听文件与 C 数组。

工作目录：

```text
E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\see_you_again_score
```

生成脚本：

```text
E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\see_you_again_score\generate_see_you_again.py
```

相关文件：

```text
see_you_again_chorus_preview.mid  电脑试听文件
see_you_again_chorus.csv          音符和持续时间明细
see_you_again_score_preview.c.txt 自动生成的C数组预览
```

脚本中的核心输入是 `SCORE`：

```python
SCORE = [
    ("A#4", 2),
    ("D5", 2),
    ("F5", 2),
    ("G5", 6),
    ("NOTE_P", 4),
]
```

每一项由“音高字符串”和“持续时间单位”组成。脚本还使用 `NOTE_TO_MIDI` 将音名映射为标准 MIDI 音高，使同一份 `SCORE` 可以同时生成电脑试听文件和固件数组。

该版本采用 78 BPM，并将每拍划分成 8 个单位：

```python
BPM = 78
UNIT_MS = 60000.0 / BPM / 8.0
```

运行方式：

```powershell
cd E:\10_ec\001_steer_sentry\agv_gimbal_cban\my_log\see_you_again_score
python generate_see_you_again.py
```

每次调整 `SCORE` 后都重新生成并试听 `see_you_again_chorus_preview.mid`。确认旋律、八度和节奏后，再把 `see_you_again_score_preview.c.txt` 中的数组复制到 `buzzer_music.c`。

#### 试听与固件之间的差异

电脑播放 MIDI 使用的是合成器音色，实际蜂鸣器只输出固定占空比的方波，因此最终仍需烧录验证：

- 某些低音在蜂鸣器上可能很弱，需要整体升高一个八度；
- 连续相同音高可能连成一个长音，需要增加 `NOTE_P`；
- 过短的音符会被 10 ms 的 `BuzzerTask()` 周期量化；
- MIDI 听感正常不代表蜂鸣器音量和辨识度一定正常；
- 临时生成文件只用于制谱，真正参与固件编译的只有复制到 `buzzer_music.c` 的 `const uint8_t` 数组。

### 2. 确定基础时间单位

乐谱中的持续时间不是毫秒，而是单位数量：

```c
NOTE_C5, 2,  // C5持续2个时间单位
NOTE_P,  1,  // 休止1个时间单位
NOTE_E5, 4,  // E5持续4个时间单位
```

每首歌可使用不同的 `note_unit_ms`。如果将一拍划分成 `N` 个单位，可按 BPM 计算：

```text
note_unit_ms = 60000 / BPM / N
```

例如 78 BPM、每拍分成 8 个单位：

```text
60000 / 78 / 8 = 96.153846 ms
```

划分越细，节奏表达越准确，但数组中的持续时间数字会更大。单个持续时间使用 `uint8_t` 保存，必须处于 `1～255`；更长的音或休止应拆成多个连续条目。

### 3. 增加音乐编号

在 `buzzer.h` 中扩展：

```c
typedef enum
{
    BUZZER_MUSIC_HARUHIKAGE = 0,
    BUZZER_MUSIC_NEW_SONG,
    BUZZER_MUSIC_COUNT
} BuzzerMusic_e;
```

`BUZZER_MUSIC_COUNT` 必须始终放在最后。

命名统一使用大写枚举名，例如：

```c
BUZZER_MUSIC_SEE_YOU_AGAIN
BUZZER_MUSIC_STARTUP
```

### 4. 声明乐谱

在 `buzzer_music.h` 中添加：

```c
extern const uint8_t buzzer_music_new_song[];
```

### 5. 编写乐谱

在 `buzzer_music.c` 中添加：

```c
const uint8_t buzzer_music_new_song[] = {
    NOTE_C5, 4,
    NOTE_E5, 4,
    NOTE_G5, 8,
    NOTE_P,  2,

    0xFF
};
```

每首乐谱必须以 `0xFF` 结束。

注意事项：

- 数组类型保持为 `const uint8_t`；
- 每个音符后必须紧跟持续时间；
- 休止符也必须带持续时间；
- `0xFF` 只作为结束标志，后面不再写持续时间；
- 音符名必须已经存在于 `BuzzerNote_e`，例如升 C 使用 `NOTE_C5S`。

### 6. 添加音乐配置

在 `buzzer.c` 的 `music_configs[]` 中添加：

```c
[BUZZER_MUSIC_NEW_SONG] = {
    .score = buzzer_music_new_song,
    .note_unit_ms = 62.5f,
    .priority = 20,
    .volume = 0.01f,
},
```

配置项含义：

- `score`：乐谱地址；
- `note_unit_ms`：该音乐一个时长单位对应的毫秒数；
- `priority`：抢占优先级，数值越大优先级越高；
- `volume`：PWM 占空比，建议先从较小值测试。

启动提示音一类非故障声音应使用较低优先级，例如 `1`；故障音乐可使用 `10` 或更高，避免启动音挡住故障报警。

### 7. 独立试听

在一个确定只执行一次的位置临时调用：

```c
BuzzerPlay(BUZZER_MUSIC_NEW_SONG, BUZZER_PLAY_ONCE);
```

`BUZZER_PLAY_ONCE` 会在播放器读到 `0xFF` 后自动停止，适合调试新乐谱。不要为了单次播放手工估算歌曲总秒数。

如果需要持续验证循环衔接，可临时使用：

```c
BuzzerPlay(BUZZER_MUSIC_NEW_SONG, BUZZER_PLAY_FOREVER);
```

测试结束后调用：

```c
BuzzerStop(BUZZER_MUSIC_NEW_SONG);
```

### 8. 编译与检查

完成上述四个源码文件的修改后执行：

```powershell
cmake --preset debug
cmake --build --preset debug
```

重点检查：

- 枚举名、数组名和配置表索引是否完全一致；
- 是否忘记在 `buzzer_music.h` 声明数组；
- 乐谱是否以 `0xFF` 结束；
- 持续时间是否超过 `255`；
- `note_unit_ms` 是否与准备乐谱时使用的节奏单位一致；
- Flash 增量是否与乐谱长度相符；
- 编译通过后再烧录试听实际蜂鸣器，高音、音量和节奏可能与电脑 MIDI 不同。

### 9. 接入上层故障

听感确认后，再在故障检测位置调用。例如周期执行的离线回调适合使用续期模式：

```c
BuzzerPlay(BUZZER_MUSIC_NEW_SONG, 1.0f);
```

只发生一次的初始化成功提示适合使用：

```c
BuzzerPlay(BUZZER_MUSIC_NEW_SONG, BUZZER_PLAY_ONCE);
```

不要在尚未验证乐谱时直接替换现有故障音乐，否则难以区分是播放器问题、调用逻辑问题还是乐谱听感问题。

## 多音乐优先级

当前虽然只有《春日影》，播放器已经预留音乐优先级。

多个音乐同时有效时：

1. 选择优先级数值最高的音乐；
2. 更高优先级音乐会立即抢占当前音乐；
3. 低优先级音乐的有效期仍按实际时间计算；
4. 高优先级音乐结束后，如果低优先级音乐仍有效，则从头播放低优先级音乐；
5. 同一首音乐重复调用只续期，不触发抢占或重新播放。

建议将严重故障配置为更高优先级。

## 常见问题

### 蜂鸣器没有声音

依次检查：

- 是否执行了 `BuzzerInit()`；
- `BuzzerTask()` 是否以约 10 ms 周期运行；
- 是否调用了 `BuzzerPlay()`；
- TIM4 CH3 和 PD14 是否配置正确；
- 音量是否过低；
- 播放时间是否已经到期。

### 音乐只响一下就停止

检查上层是否持续调用：

```c
BuzzerPlay(BUZZER_MUSIC_HARUHIKAGE, 1.0f);
```

若只调用一次，音乐本来就会在 1 秒后停止。

### 音乐一直播放

检查：

- 是否传入了 `BUZZER_PLAY_FOREVER`；
- 故障回调是否仍在周期执行；
- 故障恢复后是否仍有其他位置调用 `BuzzerPlay()`。

### 音量过大或失真

在 `music_configs[]` 中降低：

```c
.volume = 0.01f
```

占空比并非越大越好。应根据蜂鸣器和驱动电路从小到大测试。

### 音乐速度不正确

检查：

- 当前音乐配置中的 `note_unit_ms`；
- `BuzzerTask()` 的实际周期；
- 乐谱中的持续时间单位；
- 系统 DWT 是否以正确 CPU 主频初始化。

## 使用注意

- 不要在音乐播放器中使用 `HAL_Delay()`；
- 不要在中断中执行完整播放状态机；
- 上层模块只负责调用播放或停止接口；
- 新音乐必须以 `0xFF` 结束；
- 乐谱中的音符编号必须小于 `NOTE_COUNT`；
- 有限报警应持续续期，或在故障恢复时显式停止；
- 无限报警必须有明确的停止路径；
- 新增音乐后应重新全量编译并检查 Flash 占用。
