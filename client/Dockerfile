FROM node:18 AS build

WORKDIR /app

COPY package*.json ./

RUN npm install

COPY . .

ENV PORT=4200
EXPOSE 4200

CMD ["npm", "start"]
