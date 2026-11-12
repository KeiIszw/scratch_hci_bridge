"""受信済み実測の時間窓で停滞を判定する（タイマーの再読込を観測に数えない）。"""
from collections import deque


class ProgressWindow:
    def __init__(self, duration, epsilon):
        self.duration, self.epsilon = duration, epsilon
        self.samples = deque()

    def update(self, now, position):
        """窓の左端をまたぐ一標本を残し、不規則な受信周期でも期間を短縮しない。"""
        if self.samples and now < self.samples[-1][0]:
            self.samples.clear()
        self.samples.append((now, position))
        while len(self.samples) > 1 and self.samples[1][0] <= now-self.duration:
            self.samples.popleft()

    def stalled(self):
        """正味変位だけでなく全振幅を見るため、往復運動を停滞と誤認しない。"""
        return (len(self.samples) >= 2
                and self.samples[-1][0]-self.samples[0][0] >= self.duration
                and max(p for _, p in self.samples)-min(p for _, p in self.samples) <= self.epsilon)
