# Настройка Google Drive для capture_scene (rclone, headless/телефон)

`capture_scene.sh` заливает кадры на Google Drive через `rclone` (remote `gdrive:`).
Бокс `dev-workspace-1317` headless, а вход — с телефона через Cloud Shell.

⚠️ В headless Cloud Shell обычный OAuth-флоу rclone (`rclone config` → auto config)
**не завершается**: rclone не может открыть браузер, а callback идёт на
`127.0.0.1:53682`, до которого телефон не достучится (ошибка «Failed to open
browser»). Обходной путь — получить refresh-токен через **OAuth Playground**,
он целиком работает в браузере телефона, без localhost.

## A. Создать свой OAuth-клиент (GCP Console)

Ссылки ведут прямо на нужную страницу с уже выбранным проектом
**`drone-13-17-workspace-2026`** (если не залогинен — сначала вход в Google).

1. **Включить Drive API** — [Library → Google Drive API](https://console.cloud.google.com/apis/library/drive.googleapis.com?project=drone-13-17-workspace-2026)
   → кнопка **Enable**.
2. **OAuth consent screen** — [открыть](https://console.cloud.google.com/apis/credentials/consent?project=drone-13-17-workspace-2026):
   если не настроен — User Type **External**, заполнить имя/почту, добавить себя
   в **Test users**.
   **Важно:** опубликовать приложение — **Publishing status → Production**
   (иначе в режиме Testing refresh-токен умирает через 7 дней). На предупреждение
   «unverified» — просто соглашаешься.
3. **Создать OAuth client** — [Credentials](https://console.cloud.google.com/apis/credentials?project=drone-13-17-workspace-2026)
   → **Create credentials → OAuth client ID** → тип **Web application** →
   в **Authorized redirect URIs** добавить:
   `https://developers.google.com/oauthplayground`
   → Create. Скопировать **Client ID** и **Client secret**.

## B. Получить refresh-токен (OAuth Playground)

1. Открыть **[OAuth Playground](https://developers.google.com/oauthplayground)**.
2. Шестерёнка ⚙ справа → галка **«Use your own OAuth credentials»** →
   вставить Client ID + Secret.
3. Слева поле **«Input your own scopes»** → вписать:
   `https://www.googleapis.com/auth/drive`
   → **Authorize APIs** → войти своим аккаунтом → разрешить.
4. Шаг 2 → **«Exchange authorization code for tokens»** → скопировать
   **Refresh token**.

## C. Собрать rclone.conf

В Cloud Shell одной командой:

```bash
rclone config create gdrive drive \
  client_id="ВАШ_CLIENT_ID" \
  client_secret="ВАШ_SECRET" \
  scope=drive \
  token='{"access_token":"x","token_type":"Bearer","refresh_token":"ВАШ_REFRESH_TOKEN","expiry":"2000-01-01T00:00:00Z"}'
rclone lsd gdrive:        # проверка — покажет папки на Drive
```

Затем скопировать конфиг на бокс (попадёт в домашку SSH-юзера):

```bash
gcloud compute scp ~/.config/rclone/rclone.conf \
    dev-workspace-1317:~/rclone.conf \
    --zone europe-west4-a --project drone-13-17-workspace-2026
```

На боксе (под root) положить в дефолтный путь:

```bash
mkdir -p /root/.config/rclone && mv /home/*/rclone.conf /root/.config/rclone/
rclone listremotes        # должен показать gdrive:
```

После этого `make capture-scene` зальёт кадры в `gdrive:13.17/scene_img`.
