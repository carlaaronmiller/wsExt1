FROM python:3.11-slim
COPY app /app
RUN python -m pip install --verbose /app --extra-index-url https://www.piwheels.org/simple
EXPOSE 9050/tcp
LABEL version="0.0.3"
LABEL permissions='\
{\
  "ExposedPorts": {\
    "9050/tcp": {}\
  },\
  "HostConfig": {\
    "Privileged": true,\
    "Binds":["/dev:/dev","/usr/blueos/userdata/sensorData:/usr/blueos/userdata/sensorData"],\
    "PortBindings": {\
      "9050/tcp": [\
        {\
          "HostPort": "9050"\
        }\
      ]\
    }\
  }\
}'
LABEL authors='[{"name": "Carl Miller", "email": "carl.miller@dal.ca"}]'
LABEL company='{"about": "Dalhousie University", "name": "CERCOcean", "email": "carl.miller@dal.ca"}'
LABEL type="other"
LABEL readme='https://raw.githubusercontent.com/carlaaronmiller/wsExt1/blob/main/README.md'
LABEL links='{"source": "https://github.com/carlaaronmiller/wsExt1/"}'
LABEL requirements="core >= 1.1"
WORKDIR /app
ENTRYPOINT ["python", "-u", "main.py"]
