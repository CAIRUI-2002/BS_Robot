const u = document.getElementById("username");
const p = document.getElementById("password");
const r = document.getElementById("remember");
const hint = document.getElementById("hint");
const btn = document.getElementById("loginBtn");

// 恶意提示：演示用。真实项目不要明文保存密码。
(function initRemember(){
    const savedU = localStorage.getItem("demo_u");
    const savedP = localStorage.getItem("demo_p");
    if (savedU) u.value = savedU;
    if (savedP) { p.value = savedP; r.checked = true; }
})();

btn.addEventListener("click", async () => {
    hint.textContent = "";

    const username = u.value.trim();
    const password = p.value;

    if (!username || !password) {
        hint.textContent = "请输入账号和密码";
        return;
    }

    if (r.checked) {
        localStorage.setItem("demo_u", username);
        localStorage.setItem("demo_p", password);
    } else {
        localStorage.removeItem("demo_u");
        localStorage.removeItem("demo_p");
    }

    // 方式A：先做前端假登录（马上跳转）
    // window.location.href = "/home.html";

    // 方式B：调用后端接口（你看第3部分）
    try{
        const resp = await fetch("/api/login", {
            method:"POST",
            headers:{"Content-Type":"application/json"},
            body: JSON.stringify({username, password})
        });
        const data = await resp.json();
        if(!resp.ok) throw new Error(data?.message || "登录失败");
        hint.textContent = "登录成功";
        // 登录成功后跳转
        window.location.href = "/home.html";
    }catch(e){
        hint.textContent = e.message || "登录失败";
    }
});
