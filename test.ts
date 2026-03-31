// 테스트: 기본 동작 확인

// 초기화
biped.initialize()

// 버튼 A: 전진 3걸음
input.onButtonPressed(Button.A, function () {
    biped.setSpeed(biped.Speed.Normal)
    biped.walkForward(3)
})

// 버튼 B: 인사
input.onButtonPressed(Button.B, function () {
    biped.bow()
})

// 버튼 A+B: 킥
input.onButtonPressed(Button.AB, function () {
    biped.kick(biped.Leg.Right)
})
